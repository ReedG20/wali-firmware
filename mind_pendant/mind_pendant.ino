// Mind pendant firmware: XIAO ESP32-S3 Sense -> BLE audio (PROTOCOL.md v1).
// Tasks: audio (core 1, prio 23) | net: BLE notify + backlog (core 0, prio 10) | NimBLE host (core 0)
//        | Arduino loop (core 1, prio 1): serial console, control commands, NVS, button, LED.
#include "common.h"
#include <Preferences.h>
#include <esp_random.h>
#include <esp_log.h>
#include <stdarg.h>

// Default digital gain, chosen from measured mic levels (see README "Measured levels").
#define DEFAULT_GAIN 4.0f

Settings g_set;
AudioStats g_audio;
uint32_t g_boot = 0;
uint8_t g_bootTag = 0;
std::atomic<uint32_t> g_lost{0};
QueueHandle_t g_liveQ = nullptr;
QueueHandle_t g_cmdQ = nullptr;
volatile bool g_statusDirty = false, g_reqClear = false, g_reqSync = false;

static Preferences prefs;
static bool s_levels = false;
static volatile uint32_t s_logDropped = 0;

void logf(const char *fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  // Never block: drop the line if the host isn't reading or the TX buffer can't take it whole.
  if (!Serial || Serial.availableForWrite() < n) { s_logDropped++; return; }
  Serial.write((const uint8_t *)buf, n);
}

static void loadSettings() {
  prefs.begin("mind", false);
  g_set.muted = prefs.getBool("muted", false);
  g_set.mode = prefs.getUChar("mode", MODE_GATED);
  g_set.wake = prefs.getUShort("wake", 150);
  g_set.tail = prefs.getFloat("tail", 2.5f);
  g_set.gain = prefs.getFloat("gain", DEFAULT_GAIN);
  // boot id: random, and its low byte (bootTag) always differs from the previous boot's
  uint32_t prev = prefs.getULong("boot", 0);
  // 16-bit (contract v2: keeps Status short); consecutive boots always differ in the low byte.
  do { g_boot = (esp_random() ^ (uint32_t)(esp_timer_get_time() * 2654435761u)) & 0xFFFF; } while ((g_boot & 0xFF) == (prev & 0xFF) || g_boot == 0);
  prefs.putULong("boot", g_boot);
  g_bootTag = g_boot & 0xFF;
}

static void saveSettings() {
  prefs.putBool("muted", g_set.muted);
  prefs.putUChar("mode", g_set.mode);
  prefs.putUShort("wake", g_set.wake);
  prefs.putFloat("tail", g_set.tail);
  prefs.putFloat("gain", g_set.gain);
}

static void printStatus() {
  char js[200]; int n = buildStatus(js, sizeof(js));
  logf("STATUS %s (%d B)\n", js, n);
  logf("LINK conn=%d sub=%d mtu=%u itvl=%.2fms phy=%u live=%lu backfill=%lu busy=%lu connects=%lu i2sShort=%lu maxDsp=%luus logDrop=%lu heap=%u psram=%u\n",
       g_link.connected, g_link.audioSub, g_link.mtu, g_link.itvl * 1.25f, g_link.phy, (unsigned long)g_link.liveSent,
       (unsigned long)g_link.backfillSent, (unsigned long)g_link.notifyBusy, (unsigned long)g_link.connects,
       (unsigned long)g_audio.i2sShort, (unsigned long)g_audio.maxProcUs, (unsigned long)s_logDropped,
       (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
}

// Shared by BLE Control writes and the serial console. Returns false if not understood / out of range.
static bool handleCommand(char *cmd) {
  while (*cmd == ' ') cmd++;
  char *e = cmd + strlen(cmd);
  while (e > cmd && (e[-1] == '\r' || e[-1] == '\n' || e[-1] == ' ')) *--e = 0;
  char *arg = strchr(cmd, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }
  bool ok = true, save = false;
  if (!strcmp(cmd, "mute")) {
    g_set.muted = arg ? atoi(arg) != 0 : !g_set.muted; save = true;
  } else if (!strcmp(cmd, "mode") && arg && (!strcmp(arg, "all") || !strcmp(arg, "gated"))) {
    g_set.mode = !strcmp(arg, "all") ? MODE_ALL : MODE_GATED; save = true;
  } else if (!strcmp(cmd, "wake") && arg && atoi(arg) >= 10 && atoi(arg) <= 10000) {
    g_set.wake = atoi(arg); save = true;
  } else if (!strcmp(cmd, "tail") && arg && atof(arg) >= 1 && atof(arg) <= 30) {
    g_set.tail = atof(arg); save = true;
  } else if (!strcmp(cmd, "gain") && arg && atof(arg) >= 0.5 && atof(arg) <= 32) {
    g_set.gain = roundf(atof(arg) * 100) / 100; save = true;
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "sync")) {
    g_reqSync = true;
  } else if (!strcmp(cmd, "clear")) {
    g_reqClear = true;
  } else if (!strcmp(cmd, "levels")) {
    s_levels = arg ? atoi(arg) != 0 : !s_levels;
  } else if (!strcmp(cmd, "reboot")) {
    logf("rebooting\n"); delay(100); ESP.restart();
  } else if (!strcmp(cmd, "help")) {
    logf("commands: status | mute [0|1] | mode gated|all | wake N | tail S | gain G | sync | clear | levels [0|1] | reboot\n");
  } else {
    ok = false;
  }
  if (save) saveSettings();
  g_statusDirty = true;
  logf("%s cmd '%s%s%s'\n", ok ? "OK" : "ERR", cmd, arg ? " " : "", arg ? arg : "");
  return ok;
}

static void serialConsole() {
  static char line[64]; static size_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; handleCommand(line); n = 0; }
    } else if (n < sizeof(line) - 1) line[n++] = c;
  }
}

static void buttonTick() {
  static bool last = true; static uint32_t downAt = 0;
  bool up = digitalRead(PIN_BOOT_BTN);
  uint32_t now = millis();
  if (last && !up) downAt = now;
  if (!last && up) {
    uint32_t held = now - downAt;
    if (held > 30 && held < 1000) { char c[] = "mute"; handleCommand(c); }
  }
  last = up;
}

static void ledTick() {
  if (sdMounted()) return;  // GPIO21 is the SD chip select
  uint32_t t = millis();
  bool on;
  if (g_set.muted) on = false;
  else if (g_link.connected && g_link.audioSub) on = true;
  else on = (t % 2000) < 60;  // short blink while advertising
  digitalWrite(PIN_LED, on ? LOW : HIGH);  // active low
}

void setup() {
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 800) delay(10);  // brief chance for a monitor to attach; never required
  loadSettings();
  logf("\nMind pendant fw %s, contract v%d, boot=%lu tag=%u\n", FW_VERSION, CONTRACT_V, (unsigned long)g_boot, g_bootTag);
  logf("settings: muted=%d mode=%s wake=%u tail=%.1f gain=%.2f\n", g_set.muted, g_set.mode ? "all" : "gated",
       g_set.wake, g_set.tail, g_set.gain);

  // FIFO between the audio task and the net task: 250 frames (5 s), storage in PSRAM.
  static StaticQueue_t qs;
  uint8_t *qbuf = (uint8_t *)ps_malloc(250 * sizeof(Frame));
  g_liveQ = xQueueCreateStatic(250, sizeof(Frame), qbuf, &qs);
  g_cmdQ = xQueueCreate(8, 64);

  audioStart();  // seq 0 starts here
  backlogInit();
  if (!sdMounted()) { pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, HIGH); }
  bleStart();
  netStart();
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  esp_log_level_set("*", ESP_LOG_NONE);  // IDF logs could block on an unread USB console
  logf("ready. psram free=%u heap free=%u. type 'help'\n", (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap());
}

void loop() {
  static uint32_t lastBeat = 0;
  serialConsole();
  char cmd[64];
  while (xQueueReceive(g_cmdQ, cmd, 0) == pdTRUE) { logf("BLE ctrl: %s\n", cmd); handleCommand(cmd); }
  buttonTick();
  ledTick();
  audioLevelsTick(s_levels);
  if (millis() - lastBeat >= 10000) { lastBeat = millis(); printStatus(); }
  delay(10);
}

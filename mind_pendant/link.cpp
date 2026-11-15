// BLE GATT server (NimBLE) + net task.
// The net task is the only consumer of g_liveQ and the only owner of the backlog, so the single FIFO
// (backlog first, then live) needs no locking: a frame goes out live only when the phone is subscribed
// AND the backlog is empty; otherwise it is appended to the backlog (BACKFILL) and drained in order.
#include "common.h"
#include <NimBLEDevice.h>

extern QueueHandle_t g_cmdQ;  // control commands -> loop task (char[64])

LinkInfo g_link;
static NimBLEServer *s_server;
static NimBLECharacteristic *s_audio, *s_status, *s_ctrl;
static volatile uint32_t s_connAt = 0;
static volatile bool s_paramsRequested = false;

#define LIVE_LAG_FRAMES   25    // >0.5 s waiting in the live queue -> treat as backlog
#define NOTIFY_GIVEUP_MS  1000  // a single frame retried this long goes back to the backlog
#define STATUS_PERIOD_MS  2000
#define STATUS_MIN_GAP_MS 200
#define REDO_N            32    // frames re-sent after a disconnect (were in the stack, maybe never on air)

// Notifications are unacknowledged: frames accepted by the stack right before a disconnect may never have
// been transmitted. Remember the last REDO_N accepted frames and, on disconnect, put the trailing run of
// them back at the head of the FIFO (rewind the backlog for drained frames, re-insert live ones), so they
// are re-sent as BACKFILL. The phone may see a few (bootTag, seq) duplicates and keeps one copy.
static Frame s_sent[REDO_N]; static bool s_sentBL[REDO_N]; static uint32_t s_sentN = 0;

static void requeueRecent() {
  uint32_t k = s_sentN < REDO_N ? s_sentN : REDO_N;
  if (!k) return;
  bool bl = s_sentBL[(s_sentN - 1) % REDO_N];
  uint32_t run = 0;  // trailing run from the same source
  while (run < k && s_sentBL[(s_sentN - 1 - run) % REDO_N] == bl) run++;
  if (bl) backlogRewind(run);
  else {
    static Frame tmp[REDO_N];  // static: too big for the task stack
    for (uint32_t i = 0; i < run; i++) tmp[i] = s_sent[(s_sentN - run + i) % REDO_N];
    backlogUnshift(tmp, run);
  }
  logf("net: re-queued %lu recent %s frames\n", (unsigned long)run, bl ? "backlog" : "live");
  s_sentN = 0;
}

// Status JSON (PROTOCOL.md section 6). Kept compact: ~152 B typical, < 180 B worst case.
int buildStatus(char *out, size_t n) {
  return snprintf(out, n,
                  "{\"v\":%d,\"fw\":\"%s\",\"boot\":%lu,\"seq\":%lu,\"muted\":%s,\"mode\":\"%s\",\"lvl\":%u,"
                  "\"nf\":%d,\"wake\":%u,\"gain\":%g,\"buf\":%lu,\"lost\":%lu,\"sd\":%d,\"sdFree\":%lu}",
                  CONTRACT_V, FW_VERSION, (unsigned long)g_boot, (unsigned long)(g_audio.seq + 1),  // frame being captured now
                  g_set.muted ? "true" : "false", g_set.mode == MODE_ALL ? "all" : "gated", g_audio.lvl,
                  (int)lrintf(g_audio.nf), g_set.wake, (double)g_set.gain, (unsigned long)backlogCount(),
                  (unsigned long)g_lost.load(), sdMounted() ? 1 : 0, (unsigned long)sdFreeMB());
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &ci) override {
    g_link.conn = ci.getConnHandle();
    g_link.mtu = ci.getMTU();
    g_link.itvl = ci.getConnInterval();
    g_link.latency = ci.getConnLatency();
    g_link.timeout = ci.getConnTimeout();
    g_link.connected = true;
    g_link.connects++;
    s_connAt = millis();
    s_paramsRequested = false;
    s->setDataLen(ci.getConnHandle(), 251);
    s->updatePhy(ci.getConnHandle(), BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
    logf("BLE connect %s itvl=%.2fms mtu=%u\n", ci.getAddress().toString().c_str(), ci.getConnInterval() * 1.25f,
         ci.getMTU());
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &ci, int reason) override {
    g_link.connected = false; g_link.audioSub = false; g_link.statusSub = false;
    g_link.conn = 0xFFFF; g_link.mtu = 23;
    logf("BLE disconnect reason=0x%x, advertising\n", reason);
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &ci) override {
    g_link.mtu = mtu;
    logf("BLE mtu=%u\n", mtu);
    g_statusDirty = true;
  }
  void onConnParamsUpdate(NimBLEConnInfo &ci) override {
    g_link.itvl = ci.getConnInterval(); g_link.latency = ci.getConnLatency(); g_link.timeout = ci.getConnTimeout();
    logf("BLE params itvl=%.2fms latency=%u timeout=%ums\n", ci.getConnInterval() * 1.25f, ci.getConnLatency(),
         ci.getConnTimeout() * 10);
  }
  void onPhyUpdate(NimBLEConnInfo &ci, uint8_t tx, uint8_t rx) override {
    g_link.phy = tx;
    logf("BLE phy tx=%u rx=%u\n", tx, rx);
  }
};

class AudioCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &ci, uint16_t v) override {
    g_link.audioSub = (v & 1) != 0;
    logf("BLE audio %s\n", g_link.audioSub ? "subscribed" : "unsubscribed");
  }
};
class StatusCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &ci, uint16_t v) override {
    g_link.statusSub = (v & 1) != 0;
    g_statusDirty = true;
  }
  void onRead(NimBLECharacteristic *c, NimBLEConnInfo &ci) override {
    char js[200]; int n = buildStatus(js, sizeof(js));
    c->setValue((const uint8_t *)js, n);
  }
};
class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &ci) override {
    std::string v = c->getValue();
    char cmd[64]; size_t n = v.size() < 63 ? v.size() : 63;
    memcpy(cmd, v.data(), n); cmd[n] = 0;
    if (xQueueSend(g_cmdQ, cmd, 0) != pdTRUE) logf("WARN ctrl queue full\n");
  }
};

void bleStart() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(9);  // dBm, max for the S3
  NimBLEDevice::setMTU(517);
  NimBLEDevice::setDefaultPhy(BLE_GAP_LE_PHY_1M_MASK | BLE_GAP_LE_PHY_2M_MASK,
                              BLE_GAP_LE_PHY_1M_MASK | BLE_GAP_LE_PHY_2M_MASK);
  s_server = NimBLEDevice::createServer();
  s_server->setCallbacks(new ServerCB());
  s_server->advertiseOnDisconnect(true);
  NimBLEService *svc = s_server->createService(SVC_UUID);
  s_audio = svc->createCharacteristic(AUDIO_UUID, NIMBLE_PROPERTY::NOTIFY, FRAME_BYTES);
  s_status = svc->createCharacteristic(STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 200);
  s_ctrl = svc->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE, 64);
  s_audio->setCallbacks(new AudioCB());
  s_status->setCallbacks(new StatusCB());
  s_ctrl->setCallbacks(new CtrlCB());
  s_server->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad, sr;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  ad.addServiceUUID(NimBLEUUID(SVC_UUID));  // 128-bit UUID in the advertising packet
  sr.setName(DEVICE_NAME);                   // name in the scan response (does not fit with the UUID)
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->enableScanResponse(true);
  adv->setMinInterval(160);  // 100 ms
  adv->setMaxInterval(160);
  NimBLEDevice::startAdvertising();
  logf("BLE advertising as %s, addr %s\n", DEVICE_NAME, NimBLEDevice::getAddress().toString().c_str());
}

static inline bool subscribed() { return g_link.connected && g_link.audioSub && g_link.mtu >= FRAME_BYTES + 3; }

// Notify one frame; on stack congestion (no mbufs) wait and retry the same frame. Never drops.
static bool sendFrame(const Frame &f, bool fromBacklog) {
  uint32_t t0 = millis();
  int tries = 0;
  for (;;) {
    if (!subscribed()) return false;
    if (s_audio->notify(f.b, FRAME_BYTES, g_link.conn)) {
      g_link.notifyOk++;
      s_sentBL[s_sentN % REDO_N] = fromBacklog;
      s_sent[s_sentN++ % REDO_N] = f;
      return true;
    }
    g_link.notifyBusy++;
    if (millis() - t0 > NOTIFY_GIVEUP_MS) return false;
    vTaskDelay(tries++ < 4 ? 1 : 3);
  }
}

static void statusTick() {
  static uint32_t lastAt = 0, lastSig = 0;
  if (!g_link.connected) return;
  uint32_t now = millis();
  // "significant" fields for on-change notifies (lvl/seq/up change constantly -> periodic only)
  uint32_t sig = (g_set.muted ? 1 : 0) | (g_set.mode << 1) | (sdMounted() << 2) | ((backlogCount() == 0) << 3) |
                 ((uint32_t)g_set.wake << 4) ^ ((uint32_t)(g_set.gain * 100) << 12) ^ (g_lost.load() << 20);
  bool changed = sig != lastSig || g_statusDirty;
  if (!(now - lastAt >= STATUS_PERIOD_MS || (changed && now - lastAt >= STATUS_MIN_GAP_MS))) return;
  char js[200]; int n = buildStatus(js, sizeof(js));
  if (n > 160) logf("WARN status %d bytes > 160\n", n);
  s_status->setValue((const uint8_t *)js, n);
  // During a fast drain the audio notifies hold most stack buffers; if this notify fails, retry next pass.
  if (g_link.statusSub && g_link.mtu >= n + 3 && !s_status->notify((const uint8_t *)js, n, g_link.conn)) return;
  lastAt = now; lastSig = sig; g_statusDirty = false;
}

static void netTask(void *) {
  Frame f;
  for (;;) {
    if (g_reqClear) { g_reqClear = false; backlogClear(); g_statusDirty = true; }
    if (g_reqSync) { g_reqSync = false; backlogService(true); g_statusDirty = true; }
    if (g_link.connected && !s_paramsRequested && millis() - s_connAt > 1000) {
      // 15 ms: inside the 7.5-15 ms target and the one value Apple's guidelines accept for min == max.
      s_server->updateConnParams(g_link.conn, 12, 12, 0, 400);
      s_paramsRequested = true;
    }
    statusTick();

    static bool wasSub = false;
    bool sub = subscribed();
    if (wasSub && !sub) {      // phone gone
      requeueRecent();
      backlogService(true);    // persist the SD read position right away
    }
    wasSub = sub;
    if (!sub) {
      if (xQueueReceive(g_liveQ, &f, pdMS_TO_TICKS(20)) == pdTRUE) {
        backlogPush(f);
        while (xQueueReceive(g_liveQ, &f, 0) == pdTRUE) backlogPush(f);
      }
      backlogService(false);
      continue;
    }

    if (backlogEmpty() && uxQueueMessagesWaiting(g_liveQ) < LIVE_LAG_FRAMES) {
      // live path
      if (xQueueReceive(g_liveQ, &f, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (sendFrame(f, false)) g_link.liveSent++;
        else { if (!subscribed()) requeueRecent(); backlogPush(f); }  // not delivered -> head of the (empty) backlog, keeps order
      }
      continue;
    }

    // drain: everything new goes behind the backlog, then send oldest-first as fast as the stack allows
    while (xQueueReceive(g_liveQ, &f, 0) == pdTRUE) backlogPush(f);
    for (int i = 0; i < 16; i++) {
      if ((i & 7) == 7) statusTick();
      if (!backlogPeek(f)) break;
      if (!sendFrame(f, true)) break;
      backlogPop();
      g_link.backfillSent++;
    }
    backlogService(false);
    if (backlogEmpty()) g_statusDirty = true;
    taskYIELD();
  }
}

void netStart() { xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 10, nullptr, 0); }

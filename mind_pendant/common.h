// Shared definitions for the Mind pendant firmware (see ../PROTOCOL.md).
#pragma once
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define FW_VERSION  "1.1"
#define CONTRACT_V  2
#define DEVICE_NAME "Mind-Pendant"

#define SVC_UUID    "5C1B0001-9A3E-4F7B-8D2C-6E5F4A3B2C1D"
#define AUDIO_UUID  "5C1B0002-9A3E-4F7B-8D2C-6E5F4A3B2C1D"
#define STATUS_UUID "5C1B0003-9A3E-4F7B-8D2C-6E5F4A3B2C1D"
#define CTRL_UUID   "5C1B0004-9A3E-4F7B-8D2C-6E5F4A3B2C1D"

// Pins (XIAO ESP32-S3 Sense, verified against Seeed wiki)
#define PIN_PDM_CLK  42
#define PIN_PDM_DATA 41
#define PIN_SD_CS    21  // shared with the user LED (active low)
#define PIN_SD_SCK   7
#define PIN_SD_MISO  8
#define PIN_SD_MOSI  9
#define PIN_LED      21
#define PIN_BOOT_BTN 0

#define SAMPLE_RATE   16000
#define FRAME_SAMPLES 320
#define FRAME_BYTES   172
#define FRAMES_PER_S  50

#define F_SPEECH   0x01
#define F_START    0x02
#define F_END      0x04
#define F_BACKFILL 0x08

struct Frame {
  uint8_t b[FRAME_BYTES];
  uint32_t seq() const { uint32_t s; memcpy(&s, b, 4); return s; }
  uint8_t &flags() { return b[6]; }
};

enum Mode : uint8_t { MODE_GATED = 0, MODE_ALL = 1 };

// Settings: written only by the control (loop) task, read everywhere.
struct Settings {
  volatile bool muted = false;
  volatile uint8_t mode = MODE_GATED;
  volatile uint16_t wake = 150;
  volatile float tail = 2.5f;
  volatile float gain = 1.0f;  // replaced by DEFAULT_GAIN at boot if nothing stored
};
extern Settings g_set;

// Telemetry written by the audio task.
struct AudioStats {
  volatile uint32_t seq = 0;        // frame being recorded now
  volatile uint16_t lvl = 0;        // last frame RMS (pre-gain)
  volatile float nf = -90.0f;       // tracked noise floor, dBFS
  volatile uint32_t maxProcUs = 0;  // worst per-frame DSP time
  volatile uint32_t sent = 0;       // frames handed to the FIFO
  volatile uint32_t i2sShort = 0;   // short/failed I2S reads
};
extern AudioStats g_audio;

extern uint32_t g_boot;
extern uint8_t g_bootTag;
extern std::atomic<uint32_t> g_lost;
extern QueueHandle_t g_liveQ;  // audio task -> net task (single FIFO entry point)
extern volatile bool g_statusDirty;

// Requests from the control task to the net task (which owns the backlog).
extern volatile bool g_reqClear;
extern volatile bool g_reqSync;

// logging (never blocks; drops when the USB TX buffer is full or nobody reads it)
void logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// audio.cpp
void audioStart();
void audioLevelsTick(bool print);  // called 1x/s from loop for the `levels` command

// backlog.cpp (net task only, except the counters)
void backlogInit();
void backlogPush(Frame &f);
bool backlogPeek(Frame &f);
void backlogPop();
bool backlogEmpty();
void backlogService(bool force);
void backlogClear();
uint32_t backlogCount();
bool sdMounted();
uint32_t sdFreeMB();
void backlogRewind(uint32_t k);
void backlogUnshift(Frame *fs, uint32_t n);

// link.cpp
void bleStart();
void netStart();
int buildStatus(char *out, size_t n);
struct LinkInfo {
  volatile bool connected = false, audioSub = false, statusSub = false;
  volatile uint16_t conn = 0xFFFF, mtu = 23, itvl = 0, phy = 1, latency = 0, timeout = 0;
  volatile uint32_t notifyOk = 0, notifyBusy = 0, backfillSent = 0, liveSent = 0, connects = 0;
};
extern LinkInfo g_link;

// Audio capture task: PDM mic -> 20 ms frames -> HPF -> level/noise floor -> gate -> gain -> IMA ADPCM.
// Runs alone at high priority on core 1. It never blocks on anything but the I2S DMA: frames are
// handed to g_liveQ with a zero timeout (a full queue counts as `lost`, it never waits).
#include "common.h"
#include "adpcm.h"
#include <ESP_I2S.h>
#include <math.h>

static I2SClass i2s;

// 2nd-order Butterworth high-pass, RBJ cookbook, transposed direct form II.
struct Biquad {
  float b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
  void highpass(float f0, float fs) {
    float w0 = 2.0f * (float)M_PI * f0 / fs, c = cosf(w0), al = sinf(w0) / (2.0f * 0.7071f);
    float a0 = 1.0f + al;
    b0 = (1.0f + c) / 2.0f / a0; b1 = -(1.0f + c) / a0; b2 = b0;
    a1 = -2.0f * c / a0; a2 = (1.0f - al) / a0;
  }
  inline float run(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

#define HPF_HZ        90.0f
#define PREROLL_N     64        // ring size (power of 2), >= 51 frames needed
#define PREROLL_FR    50        // 1 s of pre-roll
#define WAKE_WIN      25        // 500 ms window
#define WAKE_NEED     15        // 300 ms of the window above wake
#define WARMUP_FR     25        // ignore the first 500 ms after (re)start (PDM settling)
#define NF_DOWN_ALPHA 0.10f     // floor follows quieter frames quickly (~200 ms)
#define NF_UP_DB      (0.5f / FRAMES_PER_S)  // and rises slowly, 0.5 dB/s
#define SPEECH_DB     8.0f
#define WAKE_MARGIN_DB 12.0f
#define LIM_T         16384.0f  // soft-limiter knee (-6 dBFS)

static Frame s_ring[PREROLL_N];

// Per-second level statistics for calibration (`levels` console command).
struct LevelSec {
  uint32_t seq; uint16_t minLvl, maxLvl; float meanDb; uint16_t speechFr, sentFr;
  int32_t peakIn, peakOut; uint16_t clipFr; float nf; uint32_t effWake;
};
static volatile uint32_t s_secSeq = 0;
static LevelSec s_lastSec;

static inline void putU16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

static inline void emit(Frame &f, uint32_t &lastSent) {
  lastSent = f.seq();
  if (xQueueSend(g_liveQ, &f, 0) != pdTRUE) g_lost++;
  else g_audio.sent++;
}

static void audioTask(void *) {
  static int16_t raw[FRAME_SAMPLES];
  static int16_t out[FRAME_SAMPLES];
  Biquad hpf; hpf.highpass(HPF_HZ, SAMPLE_RATE);
  ImaState ima;
  uint32_t seq = 0, lastSent = UINT32_MAX, ringValidFrom = 0, lastLoud = 0;
  uint32_t loudBits = 0;  // bit i = frame (seq - i) was loud, WAKE_WIN bits used
  bool sending = false, sendingAll = false, active = false;
  float nf = -90.0f;
  uint32_t warm = 0;
  // per-second accumulators
  uint16_t aMin = 65535, aMax = 0, aSpeech = 0, aSent = 0, aClip = 0; float aDb = 0; int aN = 0;
  int32_t aPkIn = 0, aPkOut = 0; uint32_t aWake = 0;

  for (;;) {
    size_t got = i2s.readBytes((char *)raw, sizeof(raw));
    if (got != sizeof(raw)) { g_audio.i2sShort++; if (got == 0) { vTaskDelay(1); continue; } memset((uint8_t *)raw + got, 0, sizeof(raw) - got); }
    uint32_t t0 = micros();
    g_audio.seq = seq;

    bool muted = g_set.muted;
    if (muted && !sending) {
      // Not recording. Keep reading I2S so seq stays tied to wall time.
      active = false; loudBits = 0; warm = 0; ima = ImaState(); lastLoud = 0;
      g_audio.lvl = 0;
      seq++;
      continue;
    }
    if (!active) { active = true; warm = 0; ringValidFrom = seq + WARMUP_FR; }

    // HPF + RMS (pre-gain)
    float g = g_set.gain;
    double ss = 0; int32_t pkIn = 0, pkOut = 0; bool clip = false;
    for (int i = 0; i < FRAME_SAMPLES; i++) {
      float y = hpf.run((float)raw[i]);
      ss += (double)y * y;
      int32_t yi = (int32_t)y; if (abs(yi) > pkIn) pkIn = abs(yi);
      float z = y * g;
      // soft limiter above -6 dBFS: smooth, monotonic, never reaches full scale -> no hard clipping
      float az = fabsf(z);
      if (az > LIM_T) { float o = az - LIM_T; z = copysignf(LIM_T + o / (1.0f + o / (32767.0f - LIM_T)), z); clip = true; }
      int32_t zi = (int32_t)lrintf(z);
      if (zi > 32767) zi = 32767;
      if (zi < -32768) zi = -32768;
      if (abs(zi) > pkOut) pkOut = abs(zi);
      out[i] = (int16_t)zi;
    }
    float rms = sqrtf((float)(ss / FRAME_SAMPLES));
    uint16_t level = rms > 32767.0f ? 32767 : (uint16_t)lrintf(rms);
    float db = 20.0f * log10f((level < 1 ? 1.0f : (float)level) / 32768.0f);

    // noise floor
    if (warm < WARMUP_FR) { warm++; nf = db; }
    else if (db < nf) nf += (db - nf) * NF_DOWN_ALPHA;
    else nf += NF_UP_DB;
    if (nf < -96.0f) nf = -96.0f;
    bool speech = (db >= nf + SPEECH_DB) && (db > -60.0f);
    uint32_t effWake = (uint32_t)(32768.0f * powf(10.0f, (nf + WAKE_MARGIN_DB) / 20.0f));
    if (effWake < g_set.wake) effWake = g_set.wake;
    bool loud = warm >= WARMUP_FR && level >= effWake;
    loudBits = ((loudBits << 1) | (loud ? 1 : 0)) & ((1u << WAKE_WIN) - 1);
    if (loud) lastLoud = seq;

    // build the frame
    Frame &f = s_ring[seq & (PREROLL_N - 1)];
    memcpy(f.b, &seq, 4);
    putU16(f.b + 4, level);
    f.b[6] = speech ? F_SPEECH : 0;
    f.b[7] = 1;  // IMA ADPCM
    putU16(f.b + 8, (uint16_t)(int16_t)ima.pred);
    f.b[10] = (uint8_t)ima.idx;
    f.b[11] = g_bootTag;
    imaEncode(ima, out, FRAME_SAMPLES, f.b + 12);

    // gate / send decision
    if (muted) {                       // muted while sending: close the stretch on this frame
      f.b[6] |= F_END; emit(f, lastSent); sending = sendingAll = false;
    } else if (g_set.mode == MODE_ALL) {
      if (!sending) f.b[6] |= F_START;
      sending = sendingAll = true;
      emit(f, lastSent);
    } else if (sendingAll) {           // just left `mode all`
      f.b[6] |= F_END; emit(f, lastSent); sending = sendingAll = false;
    } else if (!sending) {
      if (__builtin_popcount(loudBits) >= WAKE_NEED) {
        uint32_t first = seq >= PREROLL_FR ? seq - PREROLL_FR : 0;
        if (first < ringValidFrom) first = ringValidFrom;
        if (lastSent != UINT32_MAX && first <= lastSent) first = lastSent + 1;
        if (first > seq) first = seq;
        for (uint32_t s = first; s <= seq; s++) {
          Frame tmp = s_ring[s & (PREROLL_N - 1)];
          if (s == first) tmp.b[6] |= F_START;
          emit(tmp, lastSent);
        }
        sending = true; lastLoud = seq;
      }
    } else {
      uint32_t tailFr = (uint32_t)lrintf(g_set.tail * FRAMES_PER_S);
      if (seq - lastLoud >= tailFr) { f.b[6] |= F_END; sending = false; }
      emit(f, lastSent);
    }

    g_audio.lvl = level;
    g_audio.nf = nf;

    // per-second stats
    if (level < aMin) aMin = level;
    if (level > aMax) aMax = level;
    aDb += db; aN++;
    if (speech) aSpeech++;
    if (lastSent == seq) aSent++;
    if (clip) aClip++;
    if (pkIn > aPkIn) aPkIn = pkIn;
    if (pkOut > aPkOut) aPkOut = pkOut;
    aWake = effWake;
    if (aN >= FRAMES_PER_S) {
      s_lastSec = {seq, aMin, aMax, aDb / aN, aSpeech, aSent, aPkIn, aPkOut, aClip, nf, aWake};
      s_secSeq = seq;
      aMin = 65535; aMax = 0; aSpeech = aSent = aClip = 0; aDb = 0; aN = 0; aPkIn = aPkOut = 0;
    }

    uint32_t dt = micros() - t0;
    if (dt > g_audio.maxProcUs) g_audio.maxProcUs = dt;
    seq++;
  }
}

void audioLevelsTick(bool print) {
  static uint32_t lastSeen = 0;
  if (s_secSeq == lastSeen) return;
  lastSeen = s_secSeq;
  if (!print) return;
  LevelSec s = s_lastSec;
  logf("LVL seq=%lu min=%u max=%u meanDb=%.1f nf=%.1f effWake=%lu speech=%u sent=%u pkIn=%ld pkOut=%ld limited=%u\n",
       (unsigned long)s.seq, s.minLvl, s.maxLvl, s.meanDb, s.nf, (unsigned long)s.effWake, s.speechFr, s.sentFr,
       (long)s.peakIn, (long)s.peakOut, s.clipFr);
}

void audioStart() {
  i2s.setPinsPdmRx(PIN_PDM_CLK, PIN_PDM_DATA);
  if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    logf("ERR i2s begin failed\n");
  }
  i2s.setTimeout(200);
  xTaskCreatePinnedToCore(audioTask, "audio", 6144, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
}

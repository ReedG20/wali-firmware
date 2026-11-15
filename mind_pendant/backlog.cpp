// Store-and-forward backlog. Owned by the net task (single-threaded); other tasks only read counters.
//
// microSD layout:  /mind/NNNNNNNN.mnd  raw 172-byte frames, appended in order (max FILE_FRAMES each)
//                  /mind/head          "<fileIdx> <frameOffset>" = read position (survives reboot)
// Writes are batched in RAM and flushed every ~1 s (or when the batch is full / a drain needs it).
// Without a card: PSRAM ring of RING_FRAMES (10 min); when full the oldest frame is dropped (lost++).
#include "common.h"
#include <SD.h>
#include <SPI.h>
#include <deque>
#include <vector>
#include <algorithm>

#define FILE_FRAMES  15000   // 5 min per file (~2.6 MB)
#define WB_FRAMES    150     // RAM write batch (3 s max)
#define WB_FLUSH_MS  1000
#define RB_FRAMES    64      // read cache
#define RING_FRAMES  30000   // 10 min PSRAM ring
#define SD_MIN_FREE_MB 64
#define HEAD_SAVE_MS 1000

static bool s_sd = false;
static volatile uint32_t s_sdFreeMB = 0;
static std::atomic<uint32_t> s_count{0};

// --- SD state
struct SdFile { uint32_t idx; uint32_t frames; };
static std::deque<SdFile> s_files;   // oldest first; back() is the tail (being appended)
static uint32_t s_headOff = 0;       // frames consumed in s_files.front()
static uint32_t s_sdUnread = 0;      // frames on card not yet consumed
static Frame *s_wb = nullptr; static uint32_t s_wbN = 0; static uint32_t s_wbSince = 0;
static Frame *s_rb = nullptr; static uint32_t s_rbN = 0, s_rbPos = 0;
static uint32_t s_headSavedAt = 0; static bool s_headDirty = false;
static uint32_t s_lastFreeCheck = 0;

// --- PSRAM ring
static Frame *s_ring = nullptr; static uint32_t s_ringCap = 0, s_ringHead = 0, s_ringN = 0;

static void path(char *p, uint32_t idx) { snprintf(p, 32, "/mind/%08lu.mnd", (unsigned long)idx); }

bool sdMounted() { return s_sd; }
uint32_t sdFreeMB() { return s_sd ? s_sdFreeMB : 0; }
uint32_t backlogCount() { return s_count; }
bool backlogEmpty() { return s_count == 0; }

static void updateFree() {
  uint64_t tot = SD.totalBytes(), used = SD.usedBytes();
  s_sdFreeMB = (uint32_t)((tot > used ? tot - used : 0) >> 20);
  s_lastFreeCheck = millis();
}

static void saveHead(bool force) {
  if (!s_sd || (!s_headDirty && !force)) return;
  if (!force && millis() - s_headSavedAt < HEAD_SAVE_MS) return;
  File h = SD.open("/mind/head", FILE_WRITE);
  if (h) {
    uint32_t idx = s_files.empty() ? 0 : s_files.front().idx;
    h.printf("%lu %lu\n", (unsigned long)idx, (unsigned long)s_headOff);
    h.close();
  }
  s_headDirty = false; s_headSavedAt = millis();
}

static bool ringInit() {
  if (s_ring) return true;
  for (uint32_t cap = RING_FRAMES; cap >= 1000; cap /= 2) {
    s_ring = (Frame *)ps_malloc((size_t)cap * sizeof(Frame));
    if (s_ring) { s_ringCap = cap; logf("backlog: PSRAM ring %lu frames (%.1f min)\n", (unsigned long)cap, cap / 3000.0f); return true; }
  }
  logf("ERR backlog: no PSRAM for ring\n");
  return false;
}

static void sdFail(const char *why) {
  logf("ERR SD failure (%s): falling back to PSRAM ring; %lu frames stay on card\n", why, (unsigned long)s_sdUnread);
  s_count -= s_sdUnread; s_count -= s_wbN;
  g_lost += s_wbN;
  s_sdUnread = 0; s_wbN = 0; s_rbN = s_rbPos = 0; s_files.clear();
  s_sd = false;
  SD.end();
  ringInit();
  g_statusDirty = true;
}

static void sdMount() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, 20000000, "/sd", 5) || SD.cardType() == CARD_NONE) {
    SD.end(); SPI.end();
    logf("SD: no card\n");
    return;
  }
  s_sd = true;
  SD.mkdir("/mind");
  // scan existing backlog files
  File dir = SD.open("/mind");
  std::vector<SdFile> found;
  if (dir) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      const char *n = f.name();
      const char *base = strrchr(n, '/'); base = base ? base + 1 : n;
      unsigned long idx; char ext[8] = {0};
      if (!f.isDirectory() && sscanf(base, "%8lu.%3s", &idx, ext) == 2 && strcmp(ext, "mnd") == 0) {
        found.push_back({(uint32_t)idx, (uint32_t)(f.size() / FRAME_BYTES)});
      }
      f.close();
    }
    dir.close();
  }
  std::sort(found.begin(), found.end(), [](const SdFile &a, const SdFile &b) { return a.idx < b.idx; });
  for (auto &f : found) s_files.push_back(f);
  uint32_t hIdx = 0, hOff = 0;
  File h = SD.open("/mind/head", FILE_READ);
  if (h) { String s = h.readString(); sscanf(s.c_str(), "%lu %lu", (unsigned long *)&hIdx, (unsigned long *)&hOff); h.close(); }
  s_headOff = (!s_files.empty() && s_files.front().idx == hIdx && hOff <= s_files.front().frames) ? hOff : 0;
  uint32_t total = 0; for (auto &f : s_files) total += f.frames;
  s_sdUnread = total - s_headOff;
  // always start a fresh tail file for this boot
  uint32_t next = s_files.empty() ? 1 : s_files.back().idx + 1;
  s_files.push_back({next, 0});
  s_count = s_sdUnread;
  updateFree();
  logf("SD: mounted, type=%d size=%lluMB free=%luMB, backlog %lu frames in %u files (head off %lu)\n", SD.cardType(),
       SD.cardSize() >> 20, (unsigned long)s_sdFreeMB, (unsigned long)s_sdUnread, (unsigned)found.size(), (unsigned long)s_headOff);
}

void backlogInit() {
  s_wb = (Frame *)heap_caps_malloc(WB_FRAMES * sizeof(Frame), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s_rb = (Frame *)heap_caps_malloc(RB_FRAMES * sizeof(Frame), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  sdMount();
  if (!s_sd) ringInit();
}

// Drop the oldest SD file (card full).
static void sdDropOldest() {
  if (s_files.size() < 2) return;
  SdFile f = s_files.front();
  uint32_t n = f.frames - s_headOff;
  char p[32]; path(p, f.idx); SD.remove(p);
  s_files.pop_front(); s_headOff = 0; s_rbN = s_rbPos = 0;
  s_sdUnread -= n; s_count -= n; g_lost += n;
  s_headDirty = true;
  logf("SD: full, dropped %lu oldest frames\n", (unsigned long)n);
}

static void sdFlush() {
  if (!s_sd || s_wbN == 0) { s_wbSince = millis(); return; }
  if (millis() - s_lastFreeCheck > 30000) updateFree();
  while (s_sdFreeMB < SD_MIN_FREE_MB && s_files.size() >= 2) { sdDropOldest(); updateFree(); }
  uint32_t done = 0;
  while (done < s_wbN) {
    SdFile &t = s_files.back();
    if (t.frames >= FILE_FRAMES) { s_files.push_back({t.idx + 1, 0}); continue; }
    uint32_t n = min(s_wbN - done, (uint32_t)FILE_FRAMES - t.frames);
    char p[32]; path(p, t.idx);
    File f = SD.open(p, FILE_APPEND);
    if (!f) { sdFail("open"); return; }
    size_t w = f.write((const uint8_t *)&s_wb[done], n * sizeof(Frame));
    f.close();
    if (w != n * sizeof(Frame)) { sdFail("write"); return; }
    t.frames += n; s_sdUnread += n; done += n;
  }
  s_wbN = 0; s_wbSince = millis();
}

void backlogPush(Frame &f) {
  f.flags() |= F_BACKFILL;
  if (s_sd) {
    if (s_wbN >= WB_FRAMES) sdFlush();
    if (s_sd) {
      if (s_wbN == 0) s_wbSince = millis();
      s_wb[s_wbN++] = f; s_count++;
      return;
    }
  }
  if (!s_ring) { g_lost++; return; }
  if (s_ringN == s_ringCap) { s_ringHead = (s_ringHead + 1) % s_ringCap; s_ringN--; s_count--; g_lost++; }
  s_ring[(s_ringHead + s_ringN) % s_ringCap] = f; s_ringN++; s_count++;
}

bool backlogPeek(Frame &f) {
  if (s_sd) {
    if (s_sdUnread == 0 && s_wbN > 0) sdFlush();
    if (!s_sd) return backlogPeek(f);
    if (s_sdUnread == 0) return false;
    if (s_rbPos >= s_rbN) {
      // skip empty/consumed files at the head
      while (s_files.size() > 1 && s_headOff >= s_files.front().frames) {
        char p[32]; path(p, s_files.front().idx); SD.remove(p);
        s_files.pop_front(); s_headOff = 0; s_headDirty = true;
      }
      SdFile &h = s_files.front();
      uint32_t n = min((uint32_t)RB_FRAMES, h.frames - s_headOff);
      if (n == 0) return false;
      char p[32]; path(p, h.idx);
      File rf = SD.open(p, FILE_READ);
      if (!rf) { sdFail("read open"); return backlogPeek(f); }
      rf.seek((size_t)s_headOff * FRAME_BYTES);
      size_t r = rf.read((uint8_t *)s_rb, n * sizeof(Frame));
      rf.close();
      if (r < sizeof(Frame)) { sdFail("read"); return backlogPeek(f); }
      s_rbN = r / sizeof(Frame); s_rbPos = 0;
    }
    f = s_rb[s_rbPos];
    return true;
  }
  if (s_ringN == 0) return false;
  f = s_ring[s_ringHead];
  return true;
}

void backlogPop() {
  if (s_sd) {
    if (s_rbPos >= s_rbN || s_sdUnread == 0) return;
    s_rbPos++; s_headOff++; s_sdUnread--; s_count--; s_headDirty = true;
    if (s_headOff >= s_files.front().frames) {
      if (s_files.size() > 1) {
        char p[32]; path(p, s_files.front().idx); SD.remove(p);
        s_files.pop_front(); s_headOff = 0;
      } else if (s_wbN == 0) {
        // fully drained the tail file: delete it and start a new one
        char p[32]; path(p, s_files.front().idx); SD.remove(p);
        uint32_t next = s_files.front().idx + 1;
        s_files.clear(); s_files.push_back({next, 0}); s_headOff = 0;
      }
      s_rbN = s_rbPos = 0;
      saveHead(true);
    }
    return;
  }
  if (s_ringN == 0) return;
  s_ringHead = (s_ringHead + 1) % s_ringCap; s_ringN--; s_count--;
}

void backlogService(bool force) {
  if (s_sd) {
    if (s_wbN > 0 && (force || millis() - s_wbSince >= WB_FLUSH_MS)) sdFlush();
    saveHead(force);
    if (millis() - s_lastFreeCheck > 30000 && s_sd) updateFree();
  }
}

void backlogClear() {
  if (s_sd) {
    for (auto &f : s_files) { char p[32]; path(p, f.idx); SD.remove(p); }
    uint32_t next = s_files.empty() ? 1 : s_files.back().idx + 1;
    s_files.clear(); s_files.push_back({next, 0});
    s_headOff = 0; s_sdUnread = 0; s_wbN = 0; s_rbN = s_rbPos = 0;
    saveHead(true);
  }
  s_ringHead = s_ringN = 0;
  s_count = 0;
  logf("backlog cleared\n");
}

// Put frames that were sent right before a disconnect (maybe never transmitted) back at the FIFO head.
// Frames that came from the backlog are still in storage just before the read position: rewind.
void backlogRewind(uint32_t k) {
  if (s_sd) {
    if (s_files.empty()) return;
    uint32_t r = k < s_headOff ? k : s_headOff;  // only within the current head file
    s_headOff -= r; s_sdUnread += r; s_count += r; s_rbN = s_rbPos = 0; s_headDirty = true;
    saveHead(true);
    return;
  }
  if (!s_ring) return;
  uint32_t r = s_ringN + k <= s_ringCap ? k : s_ringCap - s_ringN;
  s_ringHead = (s_ringHead + s_ringCap - r) % s_ringCap; s_ringN += r; s_count += r;
}

// Live frames (never stored) go in front of whatever is queued. Only called when nothing is on the card
// unread (live sending implies an empty backlog), so the RAM write batch is the head of the FIFO.
void backlogUnshift(Frame *fs, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) fs[i].flags() |= F_BACKFILL;
  if (s_sd && s_sdUnread == 0) {
    if (s_wbN + n > WB_FRAMES) n = WB_FRAMES - s_wbN;
    memmove(&s_wb[n], &s_wb[0], s_wbN * sizeof(Frame));
    memcpy(&s_wb[0], fs, n * sizeof(Frame));
    if (s_wbN == 0) s_wbSince = millis();
    s_wbN += n; s_count += n;
    return;
  }
  if (s_sd || !s_ring) return;
  if (s_ringN + n > s_ringCap) n = s_ringCap - s_ringN;
  s_ringHead = (s_ringHead + s_ringCap - n) % s_ringCap;
  for (uint32_t i = 0; i < n; i++) s_ring[(s_ringHead + i) % s_ringCap] = fs[i];
  s_ringN += n; s_count += n;
}

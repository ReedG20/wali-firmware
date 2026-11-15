// Standard IMA ADPCM (WAV 0x11 tables). Exact inverse of the decoder in PROTOCOL.md section 3.
#pragma once
#include <stdint.h>

static const int16_t IMA_STEP[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,    28,
    31,    34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,   494,
    544,   598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845,  8630,
    9493,  10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
static const int8_t IMA_INDEX[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};

struct ImaState {
  int32_t pred = 0;
  int32_t idx = 0;
};

static inline uint8_t imaEncodeSample(ImaState &st, int32_t sample) {
  int32_t step = IMA_STEP[st.idx];
  int32_t diff = sample - st.pred;
  uint8_t code = 0;
  if (diff < 0) { code = 8; diff = -diff; }
  int32_t vpdiff = step >> 3;
  if (diff >= step) { code |= 4; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { code |= 2; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { code |= 1; vpdiff += step; }
  st.pred += (code & 8) ? -vpdiff : vpdiff;
  if (st.pred > 32767) st.pred = 32767;
  if (st.pred < -32768) st.pred = -32768;
  st.idx += IMA_INDEX[code];
  if (st.idx < 0) st.idx = 0;
  if (st.idx > 88) st.idx = 88;
  return code;
}

// Encodes n samples (n even) into n/2 bytes, first sample in the low nibble.
static inline void imaEncode(ImaState &st, const int16_t *in, int n, uint8_t *out) {
  for (int i = 0; i < n; i += 2) {
    uint8_t lo = imaEncodeSample(st, in[i]);
    uint8_t hi = imaEncodeSample(st, in[i + 1]);
    out[i >> 1] = lo | (hi << 4);
  }
}

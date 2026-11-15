# Mind pendant ⇄ phone: Bluetooth LE contract v2

This file is the single source of truth for how the pendant (XIAO ESP32-S3 Sense) and the iOS
app talk. It is derived from Ife's contract v0.1 (XIAO Scribe brief, section 4) with these
changes:

- Board is **BLE only** (no Wi-Fi, no on-board Scribe client).
- Advertised name is `Mind-Pendant`.
- Byte 11 of the audio frame (was reserved) is now `bootTag`.
- Store-and-forward (v0.1 "phase 2") is in v1: frames that can't be delivered go to the
  microSD card (PSRAM ring if no card) and are **drained automatically** once a phone is
  subscribed. `sync` still exists, to force a drain.
- Status gained `sd`, `sdFree`, `up` fields (`up` removed again in v2, see below).

**v2 (2026-11-14, firmware 1.1)**, requested by the app side so Status fits comfortably in one
notification at the common iOS MTU of 185 (182-byte payload):

- Status `v` is 2. Status dropped `up`; uptime is `seq × 20 ms`. `nf` is an integer (dBFS, rounded).
  `fw` is short (`"1.1"`). `boot` is now a 16-bit value (1–65535; still decode it as uint32). Typical Status
  is ~152 bytes (max ~177).
- §5: after a reconnect, the last few frames sent before the disconnect are sent again (as BACKFILL),
  because notifications are not acknowledged and those frames may never have reached the phone. The app
  sees up to ~32 duplicate `(bootTag, seq)` frames per reconnect and keeps one copy.
  Audio frame bytes are unchanged.

Whoever changes this tells the other side first and bumps `v`.

All multi-byte numbers are little-endian. One phone connects at a time. No pairing/bonding in v1.

## 1. Service

| | Value |
|---|---|
| Advertised name | `Mind-Pendant` |
| Service UUID (in advertising packet) | `5C1B0001-9A3E-4F7B-8D2C-6E5F4A3B2C1D` |

| Characteristic | UUID | Properties | Content |
|---|---|---|---|
| Audio | `5C1B0002-9A3E-4F7B-8D2C-6E5F4A3B2C1D` | Notify | One 172-byte audio frame per notification (§2) |
| Status | `5C1B0003-9A3E-4F7B-8D2C-6E5F4A3B2C1D` | Read, Notify | JSON text, ≤ 180 bytes (§5) |
| Control | `5C1B0004-9A3E-4F7B-8D2C-6E5F4A3B2C1D` | Write (with response) | One ASCII text command (§6) |

## 2. Audio frame: 172 bytes = 20 ms of 16 kHz mono audio

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | `seq` (uint32) | Frame number since boot. Frame `seq` covers the 20 ms starting `seq × 20 ms` after boot |
| 4 | 2 | `level` (uint16) | RMS (0–32767) of this frame **before** any gain. dBFS = 20·log10(level/32768) |
| 6 | 1 | `flags` | bit0 `SPEECH`, bit1 `START`, bit2 `END`, bit3 `BACKFILL` |
| 7 | 1 | `codec` | 1 = IMA ADPCM (only value in v1; 2 reserved for Opus) |
| 8 | 2 | `predictor` (int16) | ADPCM state before the first sample |
| 10 | 1 | `stepIndex` | ADPCM state before the first sample, 0–88 |
| 11 | 1 | `bootTag` | Low 8 bits of the `boot` id (§5) of the boot that **recorded** this frame |
| 12 | 160 | `codes` | 320 samples × 4 bits, first sample in the **low** nibble |

Flags:

- `SPEECH`: frame ≥ 8 dB above the tracked room noise and above −60 dBFS.
- `START`: first frame of a sent stretch (after a pause in sending).
- `END`: last frame of a sent stretch.
- `BACKFILL`: frame was stored (SD/PSRAM) because it could not be delivered live.

`seq` is the timeline. Place audio by `seq` (and `bootTag`), never by arrival time.

## 3. Compression: IMA ADPCM

Standard IMA ADPCM (same tables as WAV format 0x11). Every frame carries its own starting
state (`predictor`, `stepIndex`) so each frame decodes independently.

Decode, for each 4-bit code (low nibble of a byte first, then high nibble):

```
step = STEP[stepIndex]
diff = step >> 3
if code & 4: diff += step
if code & 2: diff += step >> 1
if code & 1: diff += step >> 2
predictor = clamp(code & 8 ? predictor - diff : predictor + diff, -32768, 32767)
stepIndex = clamp(stepIndex + INDEX[code], 0, 88)
output predictor
```

```
STEP = [7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,
        130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,
        1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
        5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,
        27086,29794,32767]
INDEX = [-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8]
```

Encoder (board): standard IMA encode, state carried continuously between frames; the header
of each frame records the state at that frame's first sample.

## 4. When the board sends

- **Gated (default).** Sends only around sound.
  - Start: sound above the wake level for 300 of the last 500 ms. Sending begins with the audio
    from 1 s earlier (pre-roll); the first frame has `START`.
  - Stop: `tail` seconds (default 2.5) after the last sound; last frame has `END`.
  - Between `END` and the next `START`, `seq` jumps. That gap is silence, not loss.
  - The wake level self-adjusts to stay ≥ 12 dB above the room noise (never below `wake`).
- **All.** Every frame (`mode all`), for debugging. First frame after entering the mode has
  `START`; `END` on leaving it.
- **Muted.** Nothing is recorded or sent. Status `muted: true`.
- **Loss.** A `seq` jump with no `END` before it and no `START` after it = frames lost.

Rate: 50 frames/s live ≈ 8.6 KB/s ≈ 31 MB per hour of sent audio.

## 5. Store-and-forward (microSD / PSRAM)

- Every frame the gate decides to send goes into one FIFO. If a phone is connected **and**
  subscribed to Audio **and** the backlog is empty, it is notified live. Otherwise it is
  appended to the backlog (microSD file if a card is mounted, else a PSRAM ring of ~10 min;
  oldest dropped when full and counted in `lost`).
- Frames written to the backlog have `BACKFILL` set.
- Once a phone is subscribed, the board drains the backlog oldest-first, faster than real time
  (as fast as notifications allow), while new frames keep appending behind it so order is
  preserved. When the backlog is empty it goes back to live.
- Notifications are unacknowledged, so frames handed to the radio right before a disconnect may be lost
  in flight. To avoid that loss, the board re-queues the last ≤ 32 frames it sent before a disconnect at
  the head of the FIFO and sends them again (with `BACKFILL`) after the next subscribe. The same
  `(bootTag, seq)` can therefore arrive twice. The app must de-duplicate (keep the first copy). This
  needs nothing extra if frames are placed by `seq` as §2 requires.
- The backlog on microSD survives reboots. Frames from an earlier boot have a `bootTag` that
  differs from the current `boot & 0xFF`; the app cannot compute their exact wall-clock time
  and should timestamp them best-effort (e.g. relative to the first frame received from them).
- The microSD CS pin is GPIO21, which is also the user LED: the LED is not used when a card is
  mounted.

## 6. Status (JSON, typically ~152 bytes, always ≤ 180)

```json
{"v":2,"fw":"1.1","boot":40213,"seq":184220,"muted":false,"mode":"gated","lvl":212,"nf":-62,"wake":150,"gain":4,"buf":0,"lost":0,"sd":1,"sdFree":28911}
```

| Field | Meaning |
|---|---|
| `v` | Contract version (this doc: 2) |
| `fw` | Firmware version (short string) |
| `boot` | Random id chosen at boot (v2: 1–65535; decode as uint32). If it changes, `seq` restarted at 0. Its low byte differs from the previous boot's |
| `seq` | Frame number being recorded right now (uptime = `seq × 20 ms`; v1's `up` was removed) |
| `muted` | Board is not recording/sending |
| `mode` | `gated` or `all` |
| `lvl` | Current loudness (RMS, same scale as `level`) |
| `nf` | Tracked room noise, dBFS, integer |
| `wake` | Loudness that starts sending (the configured value; the effective level can be higher, see §4) |
| `gain` | Digital gain applied before encoding (1 = off) |
| `buf` | Frames in the backlog not delivered yet |
| `lost` | Frames dropped since boot (backlog full, BLE errors) |
| `sd` | 1 if a microSD card is mounted |
| `sdFree` | Free space on the card in MB (0 if none) |

Board notifies Status on change and at least every 2 s while connected.

## 7. Control (text commands, write with response)

| Command | Effect |
|---|---|
| `mute 1` / `mute 0` | Stop / start recording. Same as a short press of BOOT |
| `mode gated` / `mode all` | Send only around sound / send everything |
| `wake <10–10000>` | Loudness that starts sending (quiet room ≈ 25, default 150) |
| `tail <1–30>` | Seconds sent after the last sound (default 2.5) |
| `gain <0.5–32>` | Digital gain before encoding (1 = off) |
| `status` | Notify Status now |
| `sync` | Force a backlog drain now (normally automatic) |
| `clear` | Delete the backlog (debug) |

Settings persist across restarts (NVS).

## 8. Connecting and wall-clock time

1. Scan for the service UUID, connect, discover the three characteristics.
2. Subscribe to Audio and Status, then read Status once.
3. **Time.** When Status arrives at phone time `T`, a frame with seq `s` (from the current boot)
   was recorded at `T − (status.seq − s) × 20 ms`. Valid until `boot` changes.
4. **MTU.** A frame needs 172 bytes in one notification. iOS negotiates ≥ 185 by itself.
   If the negotiated MTU is < 175, show an error.
5. **Reconnect.** On disconnect, reconnect immediately (iOS pending connects don't time out and
   work in the background).

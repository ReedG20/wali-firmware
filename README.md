# Mind pendant firmware (XIAO ESP32-S3 Sense)

BLE-only firmware that streams gated, IMA-ADPCM-compressed mic audio to the phone app and buffers to
microSD (or a PSRAM ring) while no phone is subscribed. The wire contract is **[PROTOCOL.md](PROTOCOL.md)** (v2: compact Status, possible re-sent duplicates; see its changelog).

## Build / flash / test (macOS)

```sh
brew install arduino-cli
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index && arduino-cli core install esp32:esp32@3.3.12
arduino-cli lib install NimBLE-Arduino@2.5.2
/opt/anaconda3/bin/python3 -m venv tools/.venv && tools/.venv/bin/pip install -r tools/requirements.txt

tools/flash.sh                      # compile + flash /dev/cu.usbmodem1101 (BUILD_ONLY=1 to just build)
tools/monitor.sh                    # serial console, 115200 (Ctrl-] quits)
tools/.venv/bin/python tools/ble_test.py --duration 20 --wav recordings/t.wav
tools/restore-ife-backup.sh         # put Ife's fw 1.2 back (full 8 MB image in backups/)
```

FQBN: `esp32:esp32:XIAO_ESP32S3:PSRAM=opi` (default partition scheme, 3 MB app, USB-CDC on boot).
The external antenna must be attached.

`ble_test.py` options: `--duration S`, `--cmd "mode all"` (after subscribing), `--at "10:mute 1"` (timed),
`--final-cmd`, `--wav FILE` (gaps filled with silence; `--compact` concatenates stretches instead),
`--beep-at S` (plays a 1 kHz tone through the Mac speaker as a rough end-to-end latency probe), `-v`.
It prints frames/s, the 1-second peak rate, backfill rate, seq loss vs. gated-silence gaps, duplicates and
out-of-order frames, START/END/BACKFILL/SPEECH counts, MTU, `buf` drain time and levels.

Serial console commands (same parser as the BLE Control characteristic): `status`, `mute [0|1]`,
`mode gated|all`, `wake N`, `tail S`, `gain G`, `sync`, `clear`, `levels [0|1]` (per-second level stats
for calibration), `reboot`, `help`. A STATUS/LINK line is also printed every 10 s.

## Pins (Seeed wiki + core variant file)

| Function | GPIO |
|---|---|
| PDM mic CLK / DATA | 42 / 41 |
| microSD SPI CS / SCK / MISO / MOSI | 21 / 7 / 8 / 9 |
| User LED (active low, same pin as SD CS) | 21, used only when no card is mounted |
| BOOT button (short press toggles mute) | 0 |

## Design

```
PDM mic ─DMA─► audio task (core 1, prio 23) ── g_liveQ (250 frames, 5 s, PSRAM) ──► net task (core 0, prio 10)
              HPF 90 Hz → RMS → noise floor →              never blocks:            live notify if subscribed and
              gate (1 s pre-roll, tail) → gain →           a full queue counts      backlog empty; otherwise append
              soft limiter → IMA ADPCM                     as `lost`                to backlog (BACKFILL) and drain
                                                                                     oldest-first
loop task (core 1, prio 1): serial console, Control commands, NVS, BOOT button, LED
```

* **Audio** (`audio.cpp`): `ESP_I2S` PDM RX, 16 kHz mono 16-bit, 320 samples/frame. A 2nd-order Butterworth
  high-pass at 90 Hz removes DC and rumble. `level` is the frame RMS after the HPF and before gain.
  The noise floor (dB) falls fast toward quieter frames (α=0.1/frame) and rises at 0.5 dB/s.
  `SPEECH` = ≥ 8 dB above the floor and > −60 dBFS. Gate per §4: opens when ≥ 15 of the last 25 frames are
  ≥ effective wake (= max(`wake`, floor + 12 dB)). It then sends the 1 s pre-roll (never re-sending frames
  already sent), then live frames, and ends `tail` seconds after the last loud frame. Gain is followed by a
  soft limiter (knee at −6 dBFS, asymptotic to full scale) so a loud voice right at the mic never hard-clips.
  The encoder state runs continuously and every frame header carries its own state, so pre-roll and backlog
  frames decode on their own. The first 500 ms after boot or unmute are not used for gating (PDM settling).
* **Store-and-forward** (`backlog.cpp`): a single FIFO. The net task alone owns the backlog, so no locks.
  * SD: `/mind/NNNNNNNN.mnd` files of raw 172-byte frames (5 min each). Writes are batched in RAM and
    flushed every 1 s with open-append-close, so at most ~1 s is lost on power loss.
  * The read position lives in `/mind/head` (rewritten ≤ 1×/s while draining), so the backlog survives reboot.
    Drained files are deleted. Under 64 MB free, the oldest file is dropped and counted in `lost`.
  * On any SD I/O error it falls back to the PSRAM ring.
  * No card: PSRAM ring of 30 000 frames (10 min, 5.2 MB). The oldest frame is dropped when full and counted in `lost`.
* **BLE** (`link.cpp`, NimBLE-Arduino):
  * Advertising: the 128-bit service UUID goes in the advertising packet. `Mind-Pendant` goes in the scan
    response because both don't fit in 31 bytes. Advertising interval is 100 ms and +9 dBm TX power.
  * On connect it asks for 2M PHY, 251-byte DLE and preferred MTU 517. 1 s later it requests a 15 ms
    connection interval (min = max = 15 ms). That is inside the 7.5–15 ms target and is the one
    sub-30 ms value Apple's accessory guidelines accept (min ≥ 15 ms).
  * Notify congestion (no stack buffers) never drops a frame. It waits 1–3 ms and retries the same frame.
    After 1 s without success the frame goes back to the head of the backlog.
  * If more than 0.5 s of frames are waiting in the live queue, they are moved to the backlog
    (BACKFILL), so order is always preserved.
  * Status is notified on change (rate-limited to 5/s) and every 2 s. A failed Status notify is retried, so
    `buf` stays current during a drain.
  * Advertising restarts immediately on disconnect.
* **Reconnect safety**: the last 32 frames accepted by the stack are put back at the FIFO head on disconnect
  (drained frames by rewinding the backlog read position, live frames by re-inserting), so frames that
  never left the radio are re-sent instead of lost (duplicates possible, see PROTOCOL v2 §5).
* **Logging** never blocks. `Serial.setTxTimeoutMs(0)`, and a line is dropped if `availableForWrite()` can't
  take all of it. IDF logs are silenced after setup.
* **Settings** (`muted`, `mode`, `wake`, `tail`, `gain`) persist in NVS namespace `mind`. `boot` is random
  is 16-bit (contract v2) and its low byte (`bootTag`) is forced to differ from the previous boot's.

## Measured numbers (real board, MacBook as central)

Toolchain: arduino-cli 1.6.0 (Homebrew) · esp32:esp32 **3.3.12** (ESP-IDF v5.5.5) · NimBLE-Arduino **2.5.2** ·
esptool 5.6.0 · bleak 3.1.0 · pyserial 3.5 · Python 3 (anaconda) venv. Chip: ESP32-S3 (QFN56) rev v0.2,
8 MB flash, 8 MB embedded PSRAM. Image 655 KB of 3.3 MB; 46 KB static RAM, ~166 KB heap free at runtime.

| What | Result |
|---|---|
| Audio task worst-case DSP per 20 ms frame | 1.03 ms; `i2sShort` = 0 in every run |
| BLE link (macOS central) | MTU 517 (bleak reports 515), 2M PHY, 15 ms interval accepted, 251 B DLE; connect 1.2–1.3 s |
| Live stream | 50 frames/s. 90 s `mode all` soak: 4,376 live frames, 0 lost, 0 dup, 0 out-of-order |
| Live latency (Status-based, relative to the fastest Status) | p50 1–6 ms, p95 7–16 ms after frame completion (+20 ms capture) |
| Acoustic probe (afplay launch → frame received) | 59–212 ms with the gate open. This is an upper bound: it includes afplay/CoreAudio start-up and speaker latency. 264 ms from a closed gate (the 300 ms wake rule) |
| Backfill drain (SD and PSRAM) | 288–300 frames/s ≈ **6× real time** (1 s peak 321 frames/s); 2,399 frames drained in 9.6 s |
| Store-and-forward | Speech while no client → all BACKFILL, in order, 0 loss; `buf` drains to 0 |
| Reboot persistence (SD) | Backlog from 3 earlier boots drained oldest-boot-first after reboot; read position resumed exactly |
| Disconnect mid-stream (live and drain, with and without reboot) | 0 frames missing across sessions; 9–31 duplicate `(bootTag, seq)` re-sends |
| PSRAM ring (no card) | 17,938 frames (6 min) held with `lost` = 0 |
| microSD | 128 GB SDHC, FAT32: mounted with `SD.begin(21, SPI, 20 MHz)` after `SPI.begin(7, 8, 9, 21)` |
| Status JSON | 149–151 B (contract v2) |

### Measured levels and the default gain

`levels 1` prints per-second min/max RMS, mean dBFS, noise floor, effective wake, peaks and limiter hits.

* Quiet room: frame RMS 15–60 (≈ −60 dBFS mean), noise floor −64 … −69 dBFS.
* Busy hackathon room (people talking nearby, a talk playing): noise floor −57 … −64 dBFS. The effective
  wake stays 150–170, because the floor + 12 dB is at most ~170, so the `wake` default of 150 dominates.
  Room conversation opens the gate regularly, which is correct behaviour for a wearable.
* `say` speech from the MacBook speakers (board on the desk next to it): frame RMS 100–740, mean −50 … −43 dBFS,
  sample peaks 1,000–3,000 at gain 1. The PDM mic is quiet.
* **Default gain = 4 (+12 dB).** With it, decoded speech WAVs sit at −36 … −33 dBFS RMS, peaks 22.6k–25.9k,
  0 hard-clipped samples. The soft limiter above −6 dBFS caught only the peaks. A wearer's own voice at
  ~20 cm will be louder; the limiter keeps that from hard-clipping. Tune with `gain` if needed.

### Transcription check (ElevenLabs `scribe_v3`, diarize, en)

* 28.5 s live gated clip. Scribe returned *"Remind me to call Alex about the budget meeting on Thursday at
  3:30. … The Wi-Fi password for the lab is PurpleElephant42. Sounds good. I will send the slides to Priya
  before lunch tomorrow."* That matches the spoken text word for word. Diarization split the two TTS voices
  (speaker_0 / speaker_2) and a real person in the room (speaker_1).
* An earlier clip recorded at lower speaker volume was dominated by a talk playing in the room; only
  "lunch tomorrow" of the TTS made it. That reflects the acoustic situation, not the firmware.
* Local Whisper `small.en` (`pip install faster-whisper`) gave the same words on the live and backfilled
  (SD) recordings. It is a free way to check intelligibility.

## Known limitations

* **Duplicates after reconnect.** Notifications are unacknowledged. On disconnect the last ≤ 32 frames
  handed to the stack are re-queued and re-sent as BACKFILL, so `buf` reads 32 right after a disconnect
  and the app must dedupe by `(bootTag, seq)` (PROTOCOL v2 §5).
* **Power loss.** It can lose up to ~1 s of not-yet-flushed backlog. Mid-drain it can re-send up to ~1 s of
  frames, because the head position is saved ≤ 1×/s (it is saved immediately on disconnect).
* **SD.** The card is detected at boot only (no hot-plug). After an SD I/O error the firmware falls back
  to the PSRAM ring for the rest of that boot; this path is not tested on hardware. When free space drops
  below 64 MB, the oldest 5-minute file is dropped.
* **PSRAM ring path.** Store-and-forward was verified before the card went in. The re-queue logic
  (rewind/re-insert) was verified on SD only.
* **Not tested physically:** the BOOT button short-press (`mute` over BLE and serial was tested) and the
  LED (no card → LED is used). No iPhone test was done: the 15 ms interval, 2M PHY and MTU behaviour were
  verified with macOS. iOS should behave similarly but may pick different parameters.
* **Drain speed.** About 6× real time. It is limited by how many notifications the central accepts per
  connection event; the firmware retries on congestion.
* **Gate.** `wake` in Status is the configured value. The effective wake (max(`wake`, floor + 12 dB)) is
  only visible on the serial `levels` output.
* **Bystander audio.** `recordings/` (gitignored) holds test recordings that include bystanders' speech.
  Delete them when no longer needed.

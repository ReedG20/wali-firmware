#!/usr/bin/env python3
"""Mind pendant BLE test client (macOS CoreBluetooth via bleak).

Scans for the service UUID, connects, subscribes to Audio + Status, reads Status, receives frames for
--duration seconds, decodes IMA ADPCM exactly as PROTOCOL.md section 3, and reports frames/s, seq gaps
(silence vs loss), START/END/BACKFILL counts, MTU, live latency and backlog drain. Writes a 16 kHz mono
16-bit WAV (gaps filled with silence; --compact to concatenate stretches instead).

  tools/.venv/bin/python tools/ble_test.py --duration 20 --cmd "mode all" --wav recordings/test.wav
  tools/.venv/bin/python tools/ble_test.py --duration 30 --at "10:mute 1" --at "15:mute 0"
"""
import argparse, asyncio, json, math, os, statistics, struct, subprocess, sys, tempfile, time, wave

SVC = "5c1b0001-9a3e-4f7b-8d2c-6e5f4a3b2c1d"
AUDIO = "5c1b0002-9a3e-4f7b-8d2c-6e5f4a3b2c1d"
STATUS = "5c1b0003-9a3e-4f7b-8d2c-6e5f4a3b2c1d"
CTRL = "5c1b0004-9a3e-4f7b-8d2c-6e5f4a3b2c1d"
F_SPEECH, F_START, F_END, F_BACKFILL = 1, 2, 4, 8

STEP = [7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97,
        107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
        876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871,
        5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385,
        24623, 27086, 29794, 32767]
INDEX = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8]


def decode(predictor, step_index, codes):
    out = []
    for byte in codes:
        for code in (byte & 0x0F, byte >> 4):
            step = STEP[step_index]
            diff = step >> 3
            if code & 4: diff += step
            if code & 2: diff += step >> 1
            if code & 1: diff += step >> 2
            predictor = predictor - diff if code & 8 else predictor + diff
            predictor = max(-32768, min(32767, predictor))
            step_index = max(0, min(88, step_index + INDEX[code]))
            out.append(predictor)
    return out


def parse_frame(b):
    seq, level, flags, codec, pred, sidx, tag = struct.unpack_from("<IHBBhBB", b, 0)
    return dict(seq=seq, level=level, flags=flags, codec=codec, pred=pred, sidx=sidx, tag=tag, codes=bytes(b[12:172]))


async def main(a):
    from bleak import BleakClient, BleakScanner

    t_scan = time.monotonic()
    print(f"scanning for {SVC} ...", flush=True)
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: SVC in [u.lower() for u in ad.service_uuids], timeout=a.scan_timeout)
    if not dev:
        print("ERROR: pendant not found"); return 2
    print(f"found {dev.name} {dev.address} after {time.monotonic() - t_scan:.1f}s", flush=True)

    frames, statuses, events, beeps = [], [], [], []
    tone = None
    if a.beep_at:
        tone = os.path.join(tempfile.gettempdir(), "mind_beep.wav")
        with wave.open(tone, "wb") as w:  # 1 kHz, 300 ms
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
            w.writeframes(struct.pack("<4800h", *[int(16000 * math.sin(2 * math.pi * 1000 * i / 16000)) for i in range(4800)]))
    t0 = None

    def on_audio(_, data):
        now = time.monotonic()
        if len(data) != 172:
            events.append((now, f"bad frame length {len(data)}")); return
        f = parse_frame(data); f["t"] = now
        frames.append(f)

    def on_status(_, data):
        now = time.monotonic()
        try:
            s = json.loads(bytes(data).decode()); s["_t"] = now; statuses.append(s)
            if a.verbose: print(f"  status {bytes(data).decode()}", flush=True)
        except Exception as e:
            events.append((now, f"bad status {data!r} {e}"))

    t_conn = time.monotonic()
    async with BleakClient(dev, timeout=20) as c:
        t_connected = time.monotonic()
        mtu = c.mtu_size
        print(f"connected in {t_connected - t_conn:.2f}s, mtu={mtu}", flush=True)
        if mtu < 175: print("ERROR: MTU < 175, frames will not fit")
        await c.start_notify(STATUS, on_status)
        await c.start_notify(AUDIO, on_audio)
        t0 = time.monotonic()
        st = json.loads((await c.read_gatt_char(STATUS)).decode()); st["_t"] = time.monotonic(); statuses.append(st)
        print(f"status read: {json.dumps({k: v for k, v in st.items() if k != '_t'})}", flush=True)
        for cmd in a.cmd:
            await c.write_gatt_char(CTRL, cmd.encode(), response=True); print(f"sent '{cmd}'", flush=True)
        sched = sorted((float(x.split(":", 1)[0]), x.split(":", 1)[1]) for x in a.at)
        beep_sched = sorted(a.beep_at)
        last_print = t0
        while time.monotonic() - t0 < a.duration:
            el = time.monotonic() - t0
            while sched and el >= sched[0][0]:
                _, cmd = sched.pop(0)
                await c.write_gatt_char(CTRL, cmd.encode(), response=True)
                print(f"[{el:5.1f}s] sent '{cmd}'", flush=True)
            if beep_sched and el >= beep_sched[0]:
                beep_sched.pop(0)
                beeps.append(time.monotonic())
                subprocess.Popen(["afplay", tone])
            if time.monotonic() - last_print >= a.progress:
                last_print = time.monotonic()
                s = statuses[-1] if statuses else {}
                print(f"[{el:5.1f}s] frames={len(frames)} buf={s.get('buf')} lvl={s.get('lvl')} nf={s.get('nf')} "
                      f"lost={s.get('lost')} muted={s.get('muted')} mode={s.get('mode')}", flush=True)
            await asyncio.sleep(0.05)
        if a.final_cmd:
            for cmd in a.final_cmd:
                await c.write_gatt_char(CTRL, cmd.encode(), response=True); print(f"sent '{cmd}'", flush=True)
        await asyncio.sleep(0.3)
    t_end = time.monotonic()
    return report(a, frames, statuses, events, mtu, t0, t_end, t_connected - t_conn, beeps)


def report(a, frames, statuses, events, mtu, t0, t_end, conn_s, beeps=()):
    R = {"mtu": mtu, "connect_s": round(conn_s, 2), "frames": len(frames)}
    print("\n==== report ====")
    for t, e in events: print("event:", e)
    if not frames:
        print("no frames received"); print(json.dumps(R)); return 1
    cur = statuses[-1]
    cur_tag = cur["boot"] & 0xFF
    dur = frames[-1]["t"] - frames[0]["t"]
    R["rx_window_s"] = round(dur, 2)
    R["fps_avg"] = round(len(frames) / dur, 1) if dur > 0 else None
    cnt = lambda m: sum(1 for f in frames if f["flags"] & m)
    R.update(START=cnt(F_START), END=cnt(F_END), BACKFILL=cnt(F_BACKFILL), SPEECH=cnt(F_SPEECH))
    R["codec_bad"] = sum(1 for f in frames if f["codec"] != 1)
    tags = {}
    for f in frames: tags[f["tag"]] = tags.get(f["tag"], 0) + 1
    R["bootTags"] = {str(k): v for k, v in tags.items()}
    R["current_bootTag"] = cur_tag

    # backfill phase throughput
    bf = [f for f in frames if f["flags"] & F_BACKFILL]
    if len(bf) > 10:
        bd = bf[-1]["t"] - bf[0]["t"]
        R["backfill_rx_s"] = round(bd, 2)
        R["backfill_fps"] = round(len(bf) / bd, 1) if bd > 0 else None
        R["backfill_x_realtime"] = round(len(bf) / bd / 50, 2) if bd > 0 else None
    # 1-second peak rate
    ts = [f["t"] for f in frames]; j = 0; peak = 0
    for i in range(len(ts)):
        while ts[i] - ts[j] > 1.0: j += 1
        peak = max(peak, i - j + 1)
    R["fps_peak_1s"] = peak

    # order / gaps / loss per bootTag
    loss = silence = dup = ooo = 0; gaps = []
    for tag in tags:
        fs = [f for f in frames if f["tag"] == tag]
        seen = set()
        prev = None
        for f in fs:
            if f["seq"] in seen: dup += 1; continue
            seen.add(f["seq"])
            if prev is not None:
                if f["seq"] < prev["seq"]: ooo += 1
                elif f["seq"] != prev["seq"] + 1:
                    n = f["seq"] - prev["seq"] - 1
                    if (prev["flags"] & F_END) and (f["flags"] & F_START): silence += n
                    else: loss += n; gaps.append((tag, prev["seq"], f["seq"]))
            prev = f
    R["tag_switches"] = sum(1 for i in range(1, len(frames)) if frames[i]["tag"] != frames[i - 1]["tag"])
    if a.csv:
        with open(a.csv, "w") as fh:
            fh.write("t,tag,seq,flags,level\n")
            for f in frames: fh.write(f'{f["t"] - t0:.4f},{f["tag"]},{f["seq"]},{f["flags"]},{f["level"]}\n')
    R.update(seq_lost=loss, seq_silence_gap_frames=silence, duplicates=dup, out_of_order=ooo)
    R["loss_pct"] = round(100 * loss / (len(frames) + loss), 3)
    if gaps: R["loss_gaps"] = gaps[:10]

    # latency of live frames: recorded-at from Status (T - (status.seq - s)*20ms), best (min-delay) status
    cur_st = [s for s in statuses if (s["boot"] & 0xFF) == cur_tag]
    if cur_st:
        ref = min(s["_t"] - s["seq"] * 0.02 for s in cur_st)  # phone time of seq 0 start
        lat = [f["t"] - (ref + (f["seq"] + 1) * 0.02) for f in frames
               if not (f["flags"] & F_BACKFILL) and f["tag"] == cur_tag]
        if len(lat) > 10:
            lat.sort()
            R["live_latency_ms"] = dict(p50=round(1000 * lat[len(lat) // 2]), p95=round(1000 * lat[int(len(lat) * .95)]),
                                       max=round(1000 * lat[-1]), n=len(lat))
    # acoustic end-to-end: afplay launch -> first loud live frame received (upper bound: includes afplay
    # start-up + speaker latency + 20 ms frame capture + DSP + BLE)
    bl = []
    for tb in beeps:
        before = [f["level"] for f in frames if tb - 1.0 < f["t"] < tb]
        thr = max(200, 4 * (statistics.median(before) if before else 50))
        hit = next((f for f in frames if f["t"] > tb and f["level"] > thr and not f["flags"] & F_BACKFILL), None)
        if hit: bl.append(round(1000 * (hit["t"] - tb)))
    if beeps: R["beep_to_rx_ms"] = bl
    # stretches (START..END) in arrival order: seq range, length, first-0.5 s vs peak level (pre-roll check)
    st_list, cur_s = [], None
    for f in frames:
        if f["flags"] & F_START or cur_s is None:
            cur_s = dict(tag=f["tag"], first=f["seq"], n=0, end=False, bf=bool(f["flags"] & F_BACKFILL),
                         t=round(f["t"] - t0, 2), lv=[])
            st_list.append(cur_s)
        cur_s["n"] += 1; cur_s["last"] = f["seq"]; cur_s["lv"].append(f["level"])
        if f["flags"] & F_END: cur_s["end"] = True; cur_s = None
    for x in st_list:
        lv = x.pop("lv"); x["dur_s"] = round(x["n"] * 0.02, 2)
        x["lvl_first_0.5s"] = int(statistics.mean(lv[:25])); x["lvl_max"] = max(lv)
    R["stretches"] = st_list[:20]
    bufs = [(s["_t"] - t0, s.get("buf")) for s in statuses]
    if bufs:
        R["buf_first"] = bufs[0][1]; R["buf_last"] = bufs[-1][1]
        z = [t for t, b in bufs if b == 0]
        if bufs[0][1] and z: R["buf_zero_after_s"] = round(z[0], 1)
    R["status_count"] = len(statuses)
    R["status_last"] = {k: v for k, v in cur.items() if k != "_t"}
    R["levels"] = dict(min=min(f["level"] for f in frames), max=max(f["level"] for f in frames),
                       median=int(statistics.median(f["level"] for f in frames)))

    if a.wav:
        write_wav(a, frames, cur_tag, R)
    print(json.dumps(R, indent=1))
    return 0


def write_wav(a, frames, tag, R):
    fs = sorted({f["seq"]: f for f in frames if f["tag"] == (a.wav_tag if a.wav_tag is not None else tag)}.values(),
                key=lambda f: f["seq"])
    if not fs:
        # fall back to the most common tag
        t = max(R["bootTags"], key=lambda k: R["bootTags"][k]); fs = sorted(
            {f["seq"]: f for f in frames if f["tag"] == int(t)}.values(), key=lambda f: f["seq"])
    pcm = bytearray(); prev = None; filled = 0
    for f in fs:
        if prev is not None and f["seq"] != prev + 1 and not a.compact:
            n = min(f["seq"] - prev - 1, 50 * 600)
            pcm += b"\x00\x00" * (320 * n); filled += n
        pcm += struct.pack("<320h", *decode(f["pred"], f["sidx"], f["codes"]))
        prev = f["seq"]
    with wave.open(a.wav, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(bytes(pcm))
    R["wav"] = a.wav; R["wav_s"] = round(len(pcm) / 32000, 2); R["wav_silence_filled_frames"] = filled


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=15, help="seconds to stay connected after subscribing")
    ap.add_argument("--cmd", action="append", default=[], help="control command sent right after subscribing")
    ap.add_argument("--at", action="append", default=[], help="'SECONDS:command' sent at that time")
    ap.add_argument("--final-cmd", action="append", default=[], help="command sent just before disconnecting")
    ap.add_argument("--beep-at", type=float, action="append", default=[], help="play a 1 kHz beep at T s (latency probe)")
    ap.add_argument("--wav", help="write decoded audio to this WAV")
    ap.add_argument("--csv", help="write t,tag,seq,flags,level of every received frame")
    ap.add_argument("--wav-tag", type=int, help="bootTag to write (default: current boot)")
    ap.add_argument("--compact", action="store_true", help="WAV: concatenate stretches instead of filling gaps")
    ap.add_argument("--scan-timeout", type=float, default=15)
    ap.add_argument("--progress", type=float, default=2)
    ap.add_argument("-v", "--verbose", action="store_true")
    sys.exit(asyncio.run(main(ap.parse_args())))

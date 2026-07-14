#!/usr/bin/env python3
"""Decode embedded UZC MP3 slot and compare with 441Hz reference PCM."""

import math
import re
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

SR = 44100
FREQ = 441.0
N = 1152
LEVEL = 12000
HEADER = Path(__file__).resolve().parent.parent / "src" / "uzc_mp3_slot.h"


def parse_slot_words(header_text: str) -> list[int]:
    m = re.search(r"kUzcMp3SlotWords\[.*?\]\s*=\s*\{([^}]+)\}", header_text, re.S)
    if not m:
        raise RuntimeError("slot array not found")
    return [int(x) for x in re.findall(r"-?\d+", m.group(1))]


def words_to_slot(words: list[int]) -> bytes:
    slot = bytearray()
    for w in words:
        slot.extend(struct.pack("<h", w))
    return bytes(slot)


def ref_pcm() -> list[int]:
    out = []
    phase = 0.0
    for _ in range(N):
        out.append(int(math.sin(2.0 * math.pi * phase) * LEVEL))
        phase += FREQ / SR
        if phase >= 1.0:
            phase -= 1.0
    return out


def mp3_frame_byte_length(h: bytes) -> int:
    if len(h) < 4 or h[0] != 0xFF or (h[1] & 0xE0) != 0xE0:
        return 0
    mpeg1 = ((h[1] >> 3) & 3) == 3
    br_idx = (h[2] >> 4) & 0x0F
    sr_idx = (h[2] >> 2) & 3
    br_kbps = [0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0]
    sr_mpeg1 = [44100, 48000, 32000, 0]
    sr_mpeg2 = [22050, 24000, 16000, 0]
    if br_idx == 0 or br_idx == 0x0F or sr_idx == 3:
        return 0
    bitrate = br_kbps[br_idx] * 1000
    samplerate = sr_mpeg1[sr_idx] if mpeg1 else sr_mpeg2[sr_idx]
    if bitrate == 0 or samplerate == 0:
        return 0
    padding = (h[2] >> 1) & 1
    numerator = 144 * bitrate if mpeg1 else 72 * bitrate
    return (numerator // samplerate) + padding


def decode_mp3_ffmpeg(mp3: bytes) -> list[int]:
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tf:
        tf.write(mp3)
        mp3_path = tf.name
    wav_path = mp3_path + ".wav"
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-i", mp3_path, wav_path],
            check=True,
        )
        data = Path(wav_path).read_bytes()
    finally:
        Path(mp3_path).unlink(missing_ok=True)
        Path(wav_path).unlink(missing_ok=True)
    if data[:4] != b"RIFF":
        raise RuntimeError("ffmpeg wav output missing")
    offset = 44
    pcm = []
    while offset + 1 < len(data):
        pcm.append(struct.unpack_from("<h", data, offset)[0])
        offset += 2
    return pcm


def stats(label: str, got: list[int], ref: list[int]) -> None:
    n = min(len(got), len(ref), N)
    if n == 0:
        print(f"{label}: no samples")
        return
    err = [abs(got[i] - ref[i]) for i in range(n)]
    rms_ref = math.sqrt(sum(r * r for r in ref[:n]) / n)
    rms_err = math.sqrt(sum(e * e for e in err) / n)
    print(f"{label}: samples={n} max_err={max(err)} rms_err={rms_err:.1f} rms_ref={rms_ref:.1f}")
    print(f"  first16 got : {got[:16]}")
    print(f"  first16 ref : {ref[:16]}")
    zc_got = sum(1 for i in range(1, n) if (got[i - 1] <= 0 < got[i]) or (got[i - 1] >= 0 > got[i]))
    zc_ref = sum(1 for i in range(1, n) if (ref[i - 1] <= 0 < ref[i]) or (ref[i - 1] >= 0 > ref[i]))
    print(f"  zero_cross got/ref: {zc_got}/{zc_ref} (441Hz expect ~8-9 per 1152 samples)")


def main() -> int:
    text = HEADER.read_text(encoding="utf-8")
    words = parse_slot_words(text)
    slot = words_to_slot(words)
    max_field = struct.unpack_from("<H", slot, 0)[0]
    real_field = struct.unpack_from("<H", slot, 2)[0]
    mp3_all = slot[4 : 4 + real_field]
    mp3_eff = mp3_all[: mp3_frame_byte_length(mp3_all)]
    ref = ref_pcm()

    print(f"slot_bytes={len(slot)} max={max_field} real={real_field}")
    print(f"mp3 sync={mp3_all[:4].hex()} effective_len={len(mp3_eff)} header_len={mp3_frame_byte_length(mp3_all)}")

    try:
        dec = decode_mp3_ffmpeg(mp3_eff)
        stats("ffmpeg decode (effective len)", dec, ref)
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        print(f"ffmpeg decode skipped: {e}")

    # Simulate firmware bug: pass real_field (417) always
    try:
        dec2 = decode_mp3_ffmpeg(mp3_all)
        stats("ffmpeg decode (real_field bytes)", dec2[:N], ref)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Verify a SpatialMP4 depth track: codec, compression ratio, and (optionally)
bit-exact losslessness against a reference dump of the pre-encode frames.

Usage:
  # report codec + compression ratio of the depth track
  python3 verify_depth_ffv1.py capture.mp4

  # strong end-to-end lossless check: compare every decoded depth frame against
  # the raw GRAY16LE payloads the writer fed to the encoder (see --reference).
  python3 verify_depth_ffv1.py capture.mp4 --reference <session>/depth/raw

The reference directory holds one little-endian uint16 file per frame in capture
order (e.g. frame_00000.u16 ...). Produce it with the one-time debug dump in
depth_sampler.gd (dump_raw_depth=true). Only ffmpeg/ffprobe with an FFV1 decoder
are required -- the stock system ffmpeg works; the SpatialMP4 patch is not needed
just to read depth back.
"""
import argparse, json, os, subprocess, sys, tempfile, glob

def probe_streams(path):
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-show_streams", "-of", "json", path])
    return json.loads(out)["streams"]

def pick_depth_stream(streams):
    # Depth is the square video stream tagged raw1 (legacy) or coded with ffv1.
    cands = []
    for s in streams:
        if s.get("codec_type") != "video":
            continue
        tag = s.get("codec_tag_string", "")
        name = s.get("codec_name", "")
        if tag == "raw1" or name == "ffv1":
            cands.append(s)
    if not cands:
        sys.exit("No depth stream (raw1 / ffv1) found. Streams: " +
                 ", ".join(f'{s.get("index")}:{s.get("codec_name")}/{s.get("codec_tag_string")}'
                           for s in streams if s.get("codec_type") == "video"))
    return cands[0]

def packet_bytes(path, idx):
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", str(idx),
        "-show_entries", "packet=size", "-of", "csv=p=0", path])
    return [int(x) for x in out.split() if x.strip()]

def decode_frames(path, idx, w, h, is_raw1):
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as tf:
        raw = tf.name
    if is_raw1:
        # Legacy uncompressed depth: packet bytes ARE the GRAY16LE samples, and
        # the stock ffmpeg has no decoder for the custom 'raw1' tag, so copy the
        # elementary stream out verbatim instead of decoding.
        subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-i", path,
                               "-map", f"0:{idx}", "-c", "copy", "-f", "data", raw])
    else:
        # -fps_mode passthrough is REQUIRED: depth PTS are irregular (~5 fps with
        # jitter) but the stream advertises a constant r_frame_rate, so ffmpeg's
        # default CFR conversion would duplicate/drop frames and break the
        # one-to-one match with the reference dump (false "data loss").
        subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-i", path,
                               "-map", f"0:{idx}", "-fps_mode", "passthrough",
                               "-f", "rawvideo", "-pix_fmt", "gray16le", raw])
    fb = w * h * 2
    data = open(raw, "rb").read()
    os.remove(raw)
    if len(data) % fb:
        sys.exit(f"decoded size {len(data)} not a multiple of {fb}")
    return [data[i*fb:(i+1)*fb] for i in range(len(data)//fb)]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mp4")
    ap.add_argument("--reference", help="dir of per-frame *.u16 reference dumps")
    args = ap.parse_args()

    s = pick_depth_stream(probe_streams(args.mp4))
    idx, w, h = s["index"], int(s["width"]), int(s["height"])
    name, tag = s.get("codec_name"), s.get("codec_tag_string")
    print(f"depth stream #{idx}: codec={name} tag={tag} {w}x{h}")

    sizes = packet_bytes(args.mp4, idx)
    fb = w * h * 2
    enc = sum(sizes)
    raw_total = fb * len(sizes)
    print(f"frames={len(sizes)}  raw={raw_total} B  encoded={enc} B  "
          f"ratio={raw_total/enc:.2f}x  avg={enc/len(sizes):.0f} B/frame "
          f"({'uncompressed raw1' if tag=='raw1' else name})")

    frames = decode_frames(args.mp4, idx, w, h, is_raw1=(tag == "raw1"))
    if len(frames) != len(sizes):
        print(f"WARN: decoded {len(frames)} frames but {len(sizes)} packets")

    if args.reference:
        refs = sorted(glob.glob(os.path.join(args.reference, "*.u16")))
        if len(refs) != len(frames):
            print(f"WARN: {len(refs)} reference files vs {len(frames)} decoded frames; "
                  f"comparing min({len(refs)},{len(frames)})")
        bad = 0
        n = min(len(refs), len(frames))
        for i in range(n):
            if open(refs[i], "rb").read() != frames[i]:
                bad += 1
        if bad == 0:
            print(f"LOSSLESS: all {n} frames byte-identical to the pre-encode payloads")
            sys.exit(0)
        print(f"*** DATA LOSS: {bad}/{n} frames differ from reference ***")
        sys.exit(1)
    else:
        print("(no --reference given: codec/ratio reported; for a bit-exact "
              "end-to-end check pass --reference <session>/depth/raw)")

if __name__ == "__main__":
    main()

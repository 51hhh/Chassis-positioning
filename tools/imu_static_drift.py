#!/usr/bin/env python3
"""Monitor ODOM_STATE frames and summarize static IMU yaw drift."""

from __future__ import annotations

import argparse
import math
import statistics
import time
from collections.abc import Iterable

import serial

from odom_parser import FRAME_HEADER, FRAME_LEN, parse_odom_frame

RAD_TO_DEG = 57.29577951308232


def mean(values: Iterable[float]) -> float:
    data = list(values)
    return sum(data) / len(data) if data else math.nan


def stddev(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def print_summary(
    frames: list[dict],
    frame_count: int,
    errors: int,
    started_at: float,
    label: str,
) -> None:
    elapsed = time.monotonic() - started_at
    if not frames:
        print(f"{label}: no valid ODOM_STATE frames, bytes may be missing or protocol mismatch")
        return

    yaw = [float(f["yaw_deg"]) for f in frames]
    wz = [float(f["wz"]) for f in frames]
    first_yaw = yaw[0]
    last_yaw = yaw[-1]
    yaw_delta = last_yaw - first_yaw
    yaw_span = max(yaw) - min(yaw)
    drift_rate = yaw_delta / elapsed * 60.0 if elapsed > 0 else math.nan
    hz = frame_count / elapsed if elapsed > 0 else 0.0

    print(
        f"{label}: {frame_count} frames, {hz:.1f} Hz, errors={errors}, "
        f"yaw {first_yaw:+.4f}->{last_yaw:+.4f} deg, "
        f"delta={yaw_delta:+.5f} deg, span={yaw_span:.5f} deg, "
        f"rate={drift_rate:+.5f} deg/min, "
        f"wz mean={mean(wz):+.6f} rad/s ({mean(wz) * RAD_TO_DEG:+.4f} deg/s), "
        f"wz std={stddev(wz):.6f} rad/s"
    )


def run(port: str, baud: int, duration: float, interval: float) -> int:
    ser = serial.Serial(port, baud, timeout=0.05)
    ser.reset_input_buffer()

    buf = bytearray()
    frames: list[dict] = []
    frame_count = 0
    errors = 0
    started_at = time.monotonic()
    last_report = started_at
    deadline = started_at + duration if duration > 0 else None

    print(f"Monitoring {port} @ {baud}, duration={duration if duration > 0 else 'forever'}s")
    try:
        while deadline is None or time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            while len(buf) >= FRAME_LEN:
                if buf[0:2] != FRAME_HEADER:
                    del buf[0]
                    continue
                frame = bytes(buf[:FRAME_LEN])
                parsed = parse_odom_frame(frame)
                if parsed is None:
                    errors += 1
                    del buf[0]
                    continue
                frames.append(parsed)
                frame_count += 1
                del buf[:FRAME_LEN]

            now = time.monotonic()
            if interval > 0 and now - last_report >= interval:
                print_summary(frames, frame_count, errors, started_at, "partial")
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print_summary(frames, frame_count, errors, started_at, "final")
    return 0 if frames else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to monitor, 0 = forever")
    parser.add_argument("--interval", type=float, default=10.0, help="Periodic summary interval, 0 = final only")
    args = parser.parse_args()
    return run(args.port, args.baud, args.duration, args.interval)


if __name__ == "__main__":
    raise SystemExit(main())

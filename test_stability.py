#!/usr/bin/env python3
"""
AS5048编码器长期稳定性测试

持续监控：
- 位置漂移
- 速度稳定性
- 编码器有效率
- SPI错误率（需GDB读取）

用途：验证修复后的长期可靠性
"""

import serial
import struct
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'tools'))
from odom_parser import parse_odom_frame, FRAME_HEADER, FRAME_LEN

def run_stability_test(port='/dev/ttyUSB0', baud=115200, duration_minutes=10):
    """运行稳定性测试"""

    print("=" * 80)
    print(f"AS5048编码器稳定性测试 - {duration_minutes}分钟")
    print("=" * 80)
    print(f"串口: {port} @ {baud}")
    print(f"时长: {duration_minutes}分钟")
    print()

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    buf = bytearray()
    start_time = time.time()
    end_time = start_time + duration_minutes * 60

    # 统计数据
    frame_count = 0
    enc_valid_count = 0
    parse_errors = 0

    x_min, x_max = float('inf'), float('-inf')
    y_min, y_max = float('inf'), float('-inf')
    vx_max, vy_max = 0.0, 0.0

    last_report = start_time
    report_interval = 30  # 每30秒报告一次

    print("开始监控... (Ctrl+C退出)")
    print("-" * 80)

    try:
        while time.time() < end_time:
            chunk = ser.read(128)
            if chunk:
                buf.extend(chunk)

            while len(buf) >= FRAME_LEN:
                if buf[0:2] != FRAME_HEADER:
                    del buf[0]
                    continue

                frame = bytes(buf[:FRAME_LEN])
                r = parse_odom_frame(frame)

                if r is None:
                    parse_errors += 1
                    del buf[0]
                    continue

                frame_count += 1
                if r['enc_valid']:
                    enc_valid_count += 1

                # 更新统计
                x_min = min(x_min, r['x'])
                x_max = max(x_max, r['x'])
                y_min = min(y_min, r['y'])
                y_max = max(y_max, r['y'])
                vx_max = max(vx_max, abs(r['vx']))
                vy_max = max(vy_max, abs(r['vy']))

                del buf[:FRAME_LEN]

            # 定期报告
            now = time.time()
            if now - last_report >= report_interval:
                elapsed = now - start_time
                enc_rate = 100.0 * enc_valid_count / frame_count if frame_count > 0 else 0
                hz = frame_count / elapsed if elapsed > 0 else 0

                print(f"[{int(elapsed):4d}s] 帧数={frame_count:6d} "
                      f"有效率={enc_rate:5.1f}% Hz={hz:.1f} "
                      f"x=[{x_min:+.4f},{x_max:+.4f}] "
                      f"y=[{y_min:+.4f},{y_max:+.4f}] "
                      f"vmax=({vx_max:.3f},{vy_max:.3f})")

                last_report = now

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        ser.close()

    # 最终报告
    elapsed = time.time() - start_time
    enc_rate = 100.0 * enc_valid_count / frame_count if frame_count > 0 else 0
    hz = frame_count / elapsed if elapsed > 0 else 0

    print()
    print("=" * 80)
    print("测试结果")
    print("=" * 80)
    print(f"运行时间:       {int(elapsed)}秒 ({elapsed/60:.1f}分钟)")
    print(f"总帧数:         {frame_count}")
    print(f"编码器有效帧:   {enc_valid_count}")
    print(f"有效率:         {enc_rate:.2f}%")
    print(f"解析错误:       {parse_errors}")
    print(f"平均频率:       {hz:.1f} Hz")
    print()
    print(f"X轴范围:        [{x_min:+.6f}, {x_max:+.6f}] = {(x_max-x_min)*1000:.3f}mm")
    print(f"Y轴范围:        [{y_min:+.6f}, {y_max:+.6f}] = {(y_max-y_min)*1000:.3f}mm")
    print(f"最大速度:       vx={vx_max:.3f} m/s, vy={vy_max:.3f} m/s")
    print("=" * 80)

    # 判定
    print()
    success = True
    if enc_rate < 95.0:
        print("⚠️  编码器有效率 < 95%")
        success = False
    if (x_max - x_min) > 0.002 or (y_max - y_min) > 0.002:
        print("⚠️  位置漂移 > 2mm")
        success = False
    if vx_max > 0.1 or vy_max > 0.1:
        print("⚠️  速度尖峰 > 0.1 m/s")
        success = False

    if success:
        print("✅ 稳定性测试通过")
    else:
        print("❌ 稳定性测试失败")

    return success

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='AS5048编码器稳定性测试')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--duration', type=int, default=10, help='测试时长（分钟）')

    args = parser.parse_args()

    success = run_stability_test(args.port, args.baud, args.duration)
    sys.exit(0 if success else 1)

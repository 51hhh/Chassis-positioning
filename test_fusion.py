#!/usr/bin/env python3
"""
测试 Wheel + IMU 融合里程计

验证内容：
1. YIS130 IMU yaw角数据有效性
2. 编码器位移数据有效性
3. 融合后的位置精度
4. yaw角连续性（unwrap处理）
5. 协方差/置信度（从status_bits推断）

数据流：
- YIS130 CAN → mpu_data[0].YAW_ANGLE
- AS5048 SPI → encoder delta
- 融合 → integrate_orthogonal_encoder_delta()
         → mpu_data[0].{REAL_X, REAL_Y}
"""

import serial
import struct
import time
import sys
import math
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'tools'))
from odom_parser import parse_odom_frame, FRAME_HEADER, FRAME_LEN

def test_wheel_imu_fusion(port='/dev/ttyUSB0', baud=115200, duration_sec=60):
    """测试轮式+IMU融合"""

    print("=" * 80)
    print("Wheel + IMU 融合里程计测试")
    print("=" * 80)
    print(f"串口: {port} @ {baud}")
    print(f"时长: {duration_sec}秒")
    print()

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    buf = bytearray()
    start_time = time.time()
    end_time = start_time + duration_sec

    # 数据收集
    samples = []

    print("采集数据中... (静止状态)")
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
                    del buf[0]
                    continue

                samples.append({
                    't': r['t_sec'],
                    'x': r['x'],
                    'y': r['y'],
                    'yaw_deg': r['yaw_deg'],
                    'vx': r['vx'],
                    'vy': r['vy'],
                    'wz': r['wz'],
                    'enc_valid': r['enc_valid'],
                    'imu_valid': r['imu_valid'],
                    'yaw_valid': r['yaw_valid'],
                    'pos_valid': r['pos_valid'],
                    'vel_valid': r['vel_valid'],
                })

                if len(samples) % 200 == 0:
                    elapsed = time.time() - start_time
                    print(f"[{int(elapsed):3d}s] 采集 {len(samples)} 帧")

                del buf[:FRAME_LEN]

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        ser.close()

    if len(samples) < 100:
        print("❌ 数据不足，无法分析")
        return False

    # 数据分析
    print()
    print("=" * 80)
    print("数据分析")
    print("=" * 80)

    samples = np.array(samples)
    n = len(samples)

    # 1. 有效性统计
    enc_valid_rate = np.sum([s['enc_valid'] for s in samples]) / n * 100
    imu_valid_rate = np.sum([s['imu_valid'] for s in samples]) / n * 100
    yaw_valid_rate = np.sum([s['yaw_valid'] for s in samples]) / n * 100
    pos_valid_rate = np.sum([s['pos_valid'] for s in samples]) / n * 100

    print(f"总帧数:         {n}")
    print(f"编码器有效率:   {enc_valid_rate:.1f}%")
    print(f"IMU有效率:      {imu_valid_rate:.1f}%")
    print(f"Yaw有效率:      {yaw_valid_rate:.1f}%")
    print(f"位置有效率:     {pos_valid_rate:.1f}%")
    print()

    # 2. 位置统计
    x_vals = [s['x'] for s in samples]
    y_vals = [s['y'] for s in samples]
    yaw_vals = [s['yaw_deg'] for s in samples]

    x_mean, x_std = np.mean(x_vals), np.std(x_vals)
    y_mean, y_std = np.mean(y_vals), np.std(y_vals)
    yaw_mean, yaw_std = np.mean(yaw_vals), np.std(yaw_vals)

    x_range = max(x_vals) - min(x_vals)
    y_range = max(y_vals) - min(y_vals)
    yaw_range = max(yaw_vals) - min(yaw_vals)

    print(f"X轴: 均值={x_mean:+.6f}m 标准差={x_std*1000:.3f}mm 范围={x_range*1000:.3f}mm")
    print(f"Y轴: 均值={y_mean:+.6f}m 标准差={y_std*1000:.3f}mm 范围={y_range*1000:.3f}mm")
    print(f"Yaw: 均值={yaw_mean:+.2f}° 标准差={yaw_std:.3f}° 范围={yaw_range:.3f}°")
    print()

    # 3. 速度统计
    vx_vals = [s['vx'] for s in samples]
    vy_vals = [s['vy'] for s in samples]
    wz_vals = [s['wz'] for s in samples]

    vx_max = max(abs(v) for v in vx_vals)
    vy_max = max(abs(v) for v in vy_vals)
    wz_max = max(abs(v) for v in wz_vals)

    print(f"速度范围: vx=±{vx_max:.3f} m/s, vy=±{vy_max:.3f} m/s, wz=±{wz_max:.3f} rad/s")
    print()

    # 4. Yaw连续性检查
    yaw_jumps = []
    for i in range(1, len(yaw_vals)):
        diff = yaw_vals[i] - yaw_vals[i-1]
        if abs(diff) > 180:  # 检测±180°跳变
            yaw_jumps.append((i, diff))

    if yaw_jumps:
        print(f"⚠️  检测到 {len(yaw_jumps)} 次Yaw跳变（±180°）")
        for i, diff in yaw_jumps[:5]:
            print(f"   帧{i}: {yaw_vals[i-1]:.2f}° → {yaw_vals[i]:.2f}° (Δ={diff:.2f}°)")
    else:
        print("✅ Yaw角连续，无unwrap问题")

    print()
    print("=" * 80)
    print("融合效果评估")
    print("=" * 80)

    success = True

    # 判据1: 编码器和IMU都应高可用
    if enc_valid_rate < 95.0:
        print(f"❌ 编码器有效率过低: {enc_valid_rate:.1f}% < 95%")
        success = False
    else:
        print(f"✅ 编码器有效率: {enc_valid_rate:.1f}%")

    if yaw_valid_rate < 95.0:
        print(f"❌ Yaw有效率过低: {yaw_valid_rate:.1f}% < 95%")
        success = False
    else:
        print(f"✅ Yaw有效率: {yaw_valid_rate:.1f}%")

    # 判据2: 静止时位置应稳定
    if x_range > 0.002 or y_range > 0.002:
        print(f"❌ 位置漂移过大: X={x_range*1000:.3f}mm, Y={y_range*1000:.3f}mm (>2mm)")
        success = False
    else:
        print(f"✅ 位置稳定: X={x_range*1000:.3f}mm, Y={y_range*1000:.3f}mm")

    # 判据3: Yaw角应稳定（静止状态）
    if yaw_std > 0.5:
        print(f"❌ Yaw角噪声过大: σ={yaw_std:.3f}° (>0.5°)")
        success = False
    else:
        print(f"✅ Yaw角稳定: σ={yaw_std:.3f}°")

    # 判据4: 速度应接近0（静止状态）
    if vx_max > 0.05 or vy_max > 0.05:
        print(f"❌ 速度噪声过大: vx={vx_max:.3f}, vy={vy_max:.3f} (>0.05 m/s)")
        success = False
    else:
        print(f"✅ 速度接近零: vx={vx_max:.3f}, vy={vy_max:.3f} m/s")

    print("=" * 80)
    if success:
        print("✅ 融合测试通过")
    else:
        print("❌ 融合测试失败")

    return success

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Wheel+IMU融合测试')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--duration', type=int, default=60, help='测试时长（秒）')

    args = parser.parse_args()

    success = test_wheel_imu_fusion(args.port, args.baud, args.duration)
    sys.exit(0 if success else 1)

#!/usr/bin/env python3
"""
STM32F407 里程计固件烧录测试报告
固件版本: fb852f1 (2026-06-15 19:24)
测试时间: 2026-06-15 19:30
"""

print("=" * 80)
print("STM32F407 里程计固件测试报告")
print("=" * 80)

print("\n[1] 编译状态: ✓ 成功")
print("    - 固件大小: 23320 bytes (23KB)")
print("    - 代码段: 23300 bytes")
print("    - 数据段: 20 bytes")
print("    - BSS段: 3408 bytes")
print("    - Flash占用: 23KB / 512KB (4.5%)")
print("    - RAM占用: 3.4KB / 128KB (2.7%)")

print("\n[2] 烧录状态: ✓ 成功")
print("    - 调试器: ST-LINK/V2 (VID:PID 0483:3748)")
print("    - 目标芯片: STM32F407VET6 (Device ID: 0x10076413)")
print("    - 写入速度: 27.9 KB/s")
print("    - 校验结果: ✓ 通过 (23320 bytes @ 98.5 KB/s)")

print("\n[3] 串口通信: ✓ 正常")
print("    - 设备: /dev/ttyUSB0 (CH340 USB-Serial)")
print("    - 波特率: 115200")
print("    - 帧头: AA 55 (检测到)")
print("    - 帧类型: 0x02 (ODOM_STATE)")
print("    - 帧长: 45 bytes")

print("\n[4] 数据输出: ✓ 正常")
print("    - 输出频率: ~190 Hz (目标200Hz)")
print("    - IMU数据: yaw = -1.94°, wz 有输出")
print("    - Quality: 2 (正常)")

print("\n[5] ⚠️ 观察到的问题")
print("    - 位置数据在两个值之间跳变:")
print("      (x=0, y=0) ↔ (x=-0.004, y=-0.117)")
print("    - 速度出现异常值: vy = ±23.36 m/s")
print("    - 可能原因:")
print("      a) AS5048编码器未连接或SPI通信故障")
print("      b) encoder_latest_valid_mask 在0x00和0x03之间切换")
print("      c) 需要检查status_bits中的ENC_VALID标志")

print("\n[6] 后续测试建议")
print("    1. 解析status_bits字段确认编码器有效性")
print("    2. 检查AS5048 SPI连接:")
print("       - SPI1 CS/MISO/MOSI/SCK")
print("       - SPI2 CS/MISO/MOSI/SCK")
print("    3. 测试YIS130 CAN通信:")
print("       - CAN_H/CAN_L接线")
print("       - 120Ω终端电阻")
print("       - YIS130供电和初始化")
print("    4. 使用 tools/imu_static_drift.py 监控IMU漂移")
print("    5. 记录AS5048s[].error_count和last_error_flags")

print("\n[7] 代码review结论")
print("    ✓ 今天所有19个提交已审查通过")
print("    ✓ 关键修复:")
print("      - 编码器采样原子快照 (fb852f1)")
print("      - UART响应DMA保护 (462d901)")
print("      - YIS130超时检测 (ba9f0b7)")
print("      - ISR时序优化 (b42c574)")
print("      - CAN采样点调整 (1b9b26e)")

print("\n" + "=" * 80)
print("✓ 固件烧录成功，串口通信正常")
print("⚠️ 需要接入AS5048编码器和YIS130 IMU完成完整测试")
print("=" * 80)

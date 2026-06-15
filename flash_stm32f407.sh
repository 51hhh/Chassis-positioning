#!/bin/bash
# STM32F407 烧录脚本 - 使用 OpenOCD + ST-Link

set -e

FIRMWARE_PATH="Prototype_04_407_VE_Matlab/type_04_407VE/build_gcc/type_04_407VE.bin"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$SCRIPT_DIR"

if [ ! -f "$FIRMWARE_PATH" ]; then
    echo "错误: 固件文件不存在: $FIRMWARE_PATH"
    echo "请先运行 make 编译固件"
    exit 1
fi

echo "=== STM32F407 固件烧录 ==="
echo "固件路径: $FIRMWARE_PATH"
echo "固件大小: $(du -h $FIRMWARE_PATH | cut -f1)"
echo "编译时间: $(stat -c %y $FIRMWARE_PATH | cut -d'.' -f1)"
echo ""

# 检查 ST-Link 连接
echo "正在检测 ST-Link..."
if ! lsusb | grep -i "stlink" > /dev/null 2>&1; then
    echo "警告: 未检测到 ST-Link 设备"
    echo "请检查:"
    echo "  1. ST-Link 是否连接到电脑"
    echo "  2. ST-Link 是否连接到目标板"
    echo "  3. 目标板是否上电"
    echo ""
    read -p "是否继续尝试烧录? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 使用 OpenOCD 烧录
echo ""
echo "开始烧录..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase $FIRMWARE_PATH 0x08000000" \
    -c "reset run" \
    -c "shutdown"

echo ""
echo "✓ 烧录完成"
echo ""
echo "下一步:"
echo "  1. 检查板子上的LED状态"
echo "  2. 使用 'ls /dev/ttyUSB*' 查找串口"
echo "  3. 运行: python3 tools/odom_parser.py /dev/ttyUSB0 --baud 115200"
echo "  4. 或运行: python3 tools/imu_static_drift.py /dev/ttyUSB0"

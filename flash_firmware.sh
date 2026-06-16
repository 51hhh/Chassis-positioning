#!/bin/bash
# STM32固件烧录脚本

echo "==============================================="
echo "STM32F407 固件烧录"
echo "==============================================="

FIRMWARE_PATH="/home/rick/mid360/Chassis-positioning/Prototype_04_407_VE_Matlab/type_04_407VE/build_gcc/type_04_407VE.bin"
FLASH_ADDR="0x08000000"

# 检查固件文件
if [ ! -f "$FIRMWARE_PATH" ]; then
    echo "❌ 固件文件不存在: $FIRMWARE_PATH"
    exit 1
fi

FIRMWARE_SIZE=$(stat -f%z "$FIRMWARE_PATH" 2>/dev/null || stat -c%s "$FIRMWARE_PATH")
echo "✓ 固件文件: $FIRMWARE_PATH"
echo "✓ 文件大小: $FIRMWARE_SIZE 字节"

# 检查STLink连接
echo ""
echo "检查STLink连接..."
if lsusb | grep -iq "0483"; then
    echo "✓ 检测到ST设备"
    lsusb | grep "0483"
else
    echo "❌ 未检测到STLink设备"
    echo ""
    echo "请检查："
    echo "  1. STLink已通过USB连接到电脑"
    echo "  2. STLink的SWD线已连接到STM32板"
    echo "  3. STM32板已上电"
    echo ""
    exit 1
fi

# 尝试烧录
echo ""
echo "开始烧录..."
cd "$(dirname "$FIRMWARE_PATH")/.."

openocd -f flash.cfg

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 烧录成功！"
    echo ""
    echo "下一步："
    echo "  1. 重启STM32（按复位键或重新上电）"
    echo "  2. 运行测试：bash /home/rick/mid360/kill_odom_nodes.sh"
    echo "  3. 启动RViz：docker exec -d humble bash -c 'ros2 launch wheel_imu_ekf mcu_bridge_viz.launch.py serial_port:=/dev/ttyUSB0'"
else
    echo ""
    echo "❌ 烧录失败"
    echo ""
    echo "替代方法："
    echo "  方法1 (Keil): 打开 MDK-ARM/type_04_407VE.uvprojx，按F8下载"
    echo "  方法2 (STM32CubeProgrammer): 使用GUI工具烧录 $FIRMWARE_PATH"
    exit 1
fi

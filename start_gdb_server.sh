#!/bin/bash
# STM32F407 GDB调试启动脚本

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/Prototype_04_407_VE_Matlab/type_04_407VE"

ELF_FILE="build_gcc/type_04_407VE.elf"

if [ ! -f "$ELF_FILE" ]; then
    echo "错误: ELF文件不存在: $ELF_FILE"
    echo "请先运行 make 编译固件"
    exit 1
fi

echo "=== STM32F407 GDB调试 ==="
echo "ELF文件: $ELF_FILE"
echo ""
echo "启动OpenOCD GDB服务器..."
echo "（在另一个终端运行此脚本，或按Ctrl+Z后台运行）"
echo ""

# 启动OpenOCD作为GDB服务器
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init" \
    -c "reset halt"

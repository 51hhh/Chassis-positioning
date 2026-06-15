#!/bin/bash
# 一键启动STM32 GDB调试
# 终端1运行此脚本，终端2手动启动GDB客户端

cd "$(dirname "$0")/Prototype_04_407_VE_Matlab/type_04_407VE"

echo "=== STM32F407 调试环境启动 ==="
echo ""
echo "步骤1: 启动OpenOCD GDB服务器..."
echo ""

# 启动OpenOCD（会阻塞在前台）
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# 如果OpenOCD退出
echo ""
echo "OpenOCD已停止"

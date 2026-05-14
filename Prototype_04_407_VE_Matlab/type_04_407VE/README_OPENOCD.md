# OpenOCD + ARM GCC 开发环境配置

## 概述

本项目从 Keil MDK 迁移到 GCC + OpenOCD 开源工具链，支持 Linux 下编译、烧录和调试 STM32F407VE。

## 硬件

| 项目 | 规格 |
|------|------|
| MCU | STM32F407VE (Cortex-M4 @ 168MHz, 512KB Flash, 192KB RAM) |
| 调试器 | ST-Link V2 (VID:PID 0483:3748) |
| 时钟 | HSE=25MHz, PLLM=25, PLLN=336, PLLP=2 → SYSCLK=168MHz |

## 安装依赖

```bash
# Ubuntu 24.04+
sudo apt install -y \
  gcc-arm-none-eabi \
  gdb-multiarch \
  openocd \
  make
```

验证版本：

```bash
arm-none-eabi-gcc --version   # >= 14.2.1
openocd --version             # >= 0.12.0
gdb-multiarch --version       # >= 17.1
```

## udev 规则 (ST-Link 免 sudo)

```bash
sudo tee /etc/udev/rules.d/99-stlink.rules << 'EOF'
# ST-Link V2
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", TAG+="uaccess"
# ST-Link V2-1
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", TAG+="uaccess"
# ST-Link V3
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", TAG+="uaccess"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", TAG+="uaccess"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 编译

```bash
cd Prototype_04_407_VE_Matlab/type_04_407VE
make -j$(nproc)
```

产物：
- `build_gcc/type_04_407VE.elf` — 调试用
- `build_gcc/type_04_407VE.hex` — 烧录用
- `build_gcc/type_04_407VE.bin` — 二进制固件

清理：`make clean`

## 烧录

```bash
openocd -f openocd.cfg -c "init; halt; program build_gcc/type_04_407VE.elf verify; reset run; exit"
```

## 调试 (GDB)

终端 1 — 启动 OpenOCD 服务：

```bash
openocd -f openocd.cfg
```

终端 2 — 连接 GDB：

```bash
gdb-multiarch build_gcc/type_04_407VE.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) break main
(gdb) continue
```

## OpenOCD 配置说明

`openocd.cfg` 内容：

```
source [find interface/stlink.cfg]
source [find target/stm32f4x.cfg]
adapter speed 4000
reset_config none
```

- `adapter speed 4000` — SWD 时钟 4MHz，稳定可靠
- `reset_config none` — 不使用硬件 SRST 引脚（板子未接该线），改用软件复位

## HardFault 调试方法

当遇到 HardFault 时，通过 GDB 读取故障寄存器：

```gdb
(gdb) monitor reset halt
(gdb) x/x 0xE000ED28    # CFSR (Configurable Fault Status Register)
(gdb) x/x 0xE000ED38    # BFAR (Bus Fault Address Register)
(gdb) x/x 0xE000ED34    # MMFAR (MemManage Fault Address Register)
(gdb) info reg           # 查看通用寄存器
(gdb) bt                 # 回溯调用栈
```

CFSR 位域：
- Bit [25] DIVBYZERO — 除零错误
- Bit [10] IMPRECISERR — 不精确总线错误
- Bit [9] PRECISERR — 精确总线错误（BFAR 有效）
- Bit [0] IBUSERR — 取指总线错误

## 项目结构

```
type_04_407VE/
├── Makefile                   # GCC 构建脚本
├── openocd.cfg                # OpenOCD 配置
├── STM32F407VETx_FLASH.ld     # GCC 链接脚本
├── Core/                      # STM32CubeMX 生成代码
│   ├── Inc/                   # 头文件
│   └── Src/                   # main.c, spi.c, tim.c 等
├── MDK-ARM/                   # 应用层代码
│   ├── AS5048.c/h             # AS5048A 磁编码器 SPI 驱动
│   ├── IM_TEST.c/h            # Simulink 生成的运动学模型
│   ├── YIS130.c/h             # YIS130 IMU 驱动
│   └── odom_protocol.c/h     # 里程计二进制通信协议
├── Drivers/                   # HAL 库 + CMSIS
└── build_gcc/                 # 编译输出 (git ignored)
```

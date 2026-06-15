# STM32F407 编码器调试文档

## 快速开始

### 方法1：自动启动（推荐，需要tmux）
```bash
cd /home/rick/mid360/Chassis-positioning
./start_debug.sh
```

### 方法2：手动启动

**终端1 - 启动OpenOCD服务器**
```bash
cd /home/rick/mid360/Chassis-positioning
./debug_session.sh
```

**终端2 - 启动GDB客户端**
```bash
cd /home/rick/mid360/Chassis-positioning/Prototype_04_407_VE_Matlab/type_04_407VE
gdb-multiarch -x debug_stm32.gdb build_gcc/type_04_407VE.elf
```

## 快速诊断

### 步骤1：查看编码器状态
```gdb
(gdb) p AS5048s[0]
(gdb) p AS5048s[1]
```

检查字段：
- `valid`: 0=故障, 1=正常
- `error_count`: 累计错误次数
- `last_error_flags`: 错误类型
  - `0xFE` = SPI第1次传输超时（MISO无信号）
  - `0xFD` = SPI第2次传输超时
  - `0xFC` = 奇偶校验失败
  - `0x01-0x07` = AS5048硬件错误

### 步骤2：自动采集100次数据

在GDB中粘贴以下完整脚本：
```gdb
set pagination off
set logging file encoder_debug.log
set logging on
b service_encoder_sampling
commands
silent
printf "[%d] AS5048[0]: valid=%d err_cnt=%d err_flags=0x%02X angle=%d delta=%d\n", $n++, AS5048s[0].valid, AS5048s[0].error_count, AS5048s[0].last_error_flags, AS5048s[0].angle, AS5048s[0].delta_dis
printf "[%d] AS5048[1]: valid=%d err_cnt=%d err_flags=0x%02X angle=%d delta=%d\n", $n, AS5048s[1].valid, AS5048s[1].error_count, AS5048s[1].last_error_flags, AS5048s[1].angle, AS5048s[1].delta_dis
c
end
set $n = 0
c 100
set logging off
quit
```

查看结果：
```bash
cat encoder_debug.log
```

## 故障排查

### 问题1：last_error_flags = 0xFE 或 0xFD
**原因**: SPI超时，MISO引脚无数据
**检查**:
- AS5048供电是否正常（3.3V）
- MISO引脚连接
- SPI1/SPI2 CS引脚电平

### 问题2：last_error_flags = 0xFC
**原因**: 奇偶校验失败
**检查**:
- SPI时序（可能速率过快）
- 磁铁位置（是否对中）
- 接线质量（EMI干扰）

### 问题3：valid间歇性为0
**原因**: 硬件接触不良
**检查**:
- 杜邦线质量
- 插针接触
- 电源稳定性

### 问题4：AS5048硬件错误 (0x01-0x07)
**原因**: 磁场强度或AGC异常
**检查**:
- 磁铁距离（推荐0.5-3mm）
- 磁铁对中（不要偏心）
- 读取AGC寄存器值

## 高级调试

### 查看SPI寄存器
```gdb
(gdb) p/x *((SPI_TypeDef*)0x40013000)  # SPI1
(gdb) p/x *((SPI_TypeDef*)0x40003800)  # SPI2
```

### 单步调试SPI读取
```gdb
(gdb) b AS5048_Read
(gdb) c
(gdb) n  # 单步执行
(gdb) p response  # 查看SPI返回值
```

### 实时监控变量
```gdb
(gdb) watch encoder_latest_valid_mask
(gdb) c  # 当变量改变时自动停止
```

## 文件说明

- `debug_session.sh` - 启动OpenOCD服务器
- `start_debug.sh` - 一键启动（tmux自动分割终端）
- `DEBUG_GUIDE.sh` - 显示完整调试指南
- `debug_stm32.gdb` - GDB配置文件
- `encoder_debug.log` - 自动诊断日志输出

## 退出调试

```gdb
(gdb) quit
```

终端1按 `Ctrl+C` 停止OpenOCD

如果使用tmux: `Ctrl+B` 然后 `D` 分离会话，`tmux kill-session -t stm32_debug` 彻底关闭

#!/bin/bash
# STM32F407 编码器调试快速指南

cat << 'EOF'
╔═══════════════════════════════════════════════════════════════╗
║          STM32F407 编码器问题 GDB 调试指南                    ║
╚═══════════════════════════════════════════════════════════════╝

【问题现象】
  位置在 (0,0) 和 (-0.004, -0.117) 之间跳变
  速度出现 ±23.36 m/s 异常值

【可能原因】
  1. AS5048 SPI通信失败（MISO无数据）
  2. 编码器奇偶校验错误
  3. 编码器硬件故障标志
  4. SPI时序问题

【调试步骤】

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
终端1: 启动OpenOCD GDB服务器
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

cd /home/rick/mid360/Chassis-positioning
./debug_session.sh

（保持此终端运行，OpenOCD会监听3333端口）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
终端2: 启动GDB客户端
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

cd /home/rick/mid360/Chassis-positioning/Prototype_04_407_VE_Matlab/type_04_407VE
gdb-multiarch -x debug_stm32.gdb build_gcc/type_04_407VE.elf

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GDB 关键调试命令
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【1. 检查编码器状态】
  (gdb) p AS5048s[0]
  (gdb) p AS5048s[1]

  重点关注:
    .valid           - 是否有效 (0=故障, 1=正常)
    .error_count     - 累计错误次数
    .last_error_flags - 最后一次错误标志
                       0xFE = SPI第1次传输超时
                       0xFD = SPI第2次传输超时
                       0xFC = 奇偶校验失败
                       0x01-0x07 = AS5048硬件错误
    .delta_dis       - 增量位移
    .angle           - 当前角度

【2. 在SPI读取处设置断点】
  (gdb) b AS5048_Read
  (gdb) c

  触发后查看:
  (gdb) p *AS5
  (gdb) p registerAddress
  (gdb) n    # 单步执行
  (gdb) p response   # 查看SPI返回值

【3. 在编码器采样处设置断点】
  (gdb) b service_encoder_sampling
  (gdb) c

  触发后:
  (gdb) p AS5048s[0].valid
  (gdb) p AS5048s[1].valid
  (gdb) n    # 单步执行，观察valid标志变化

【4. 监控快照变量】
  (gdb) watch encoder_delta_x_counts
  (gdb) watch encoder_delta_y_counts
  (gdb) watch encoder_latest_valid_mask
  (gdb) c    # 当这些变量改变时会自动停止

【5. 查看SPI寄存器】
  (gdb) p/x *((SPI_TypeDef*)0x40013000)  # SPI1
  (gdb) p/x *((SPI_TypeDef*)0x40003800)  # SPI2

  关注:
    SR  - 状态寄存器 (bit6=BSY, bit0=RXNE)
    DR  - 数据寄存器

【6. 实时打印循环】
  (gdb) b service_encoder_sampling
  (gdb) commands
  > silent
  > printf "[ENC] valid_mask=0x%02X  x_counts=%d  y_counts=%d\n", valid_mask, AS5048s[0].delta_dis, AS5048s[1].delta_dis
  > c
  > end
  (gdb) c    # 每次调用都会打印，不停止

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
典型故障模式分析
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【模式1: MISO无信号】
  现象: last_error_flags = 0xFE 或 0xFD (SPI超时)
  原因: MISO引脚未连接或AS5048未上电
  解决: 检查硬件连接，万用表测量3.3V供电

【模式2: 奇偶校验失败】
  现象: last_error_flags = 0xFC
  原因: SPI时序问题或AS5048磁铁位置异常
  解决: 降低SPI速率，检查磁铁对中

【模式3: AS5048硬件错误】
  现象: last_error_flags = 0x01~0x07
  原因: 磁场强度不足、AGC超范围、CORDIC溢出
  解决: 调整磁铁距离(0.5-3mm)

【模式4: valid间歇性为0】
  现象: valid在0/1之间切换，导致位置跳变
  原因: 接触不良、EMI干扰、SPI时序裕量不足
  解决: 检查杜邦线质量，加滤波电容，降低SPI速率

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
快速诊断脚本
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

在GDB中粘贴以下命令，自动采集100次数据:

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

查看日志:
  cat encoder_debug.log

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
退出调试
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  (gdb) quit
  终端1: Ctrl+C 停止OpenOCD

EOF

# ODOM 固件当前实现记录

## 当前数据链路

YIS130 通过 CAN1 接入 STM32F407，SN65HVD230DR 负责 CAN 收发。YIS130 使用 SAE J1939 兼容扩展帧，默认 500 kbit/s、200 Hz 输出。

AS5048A 磁编码器通过 SPI1/SPI2 读取，当前用于正交 tracking wheel 里程计：

- SPI1/AS5048 index 0：默认 body X/前进 tracking wheel
- SPI2/AS5048 index 1：默认 body Y/左向 tracking wheel
- 编码器采样请求由 TIM11 20 kHz ISR 分频到 1 kHz 产生
- 阻塞式 SPI 读取在 main loop 执行，delta 和 valid 状态快照后再交给 ISR 消费

## 串口协议

UART1 115200 8N1 输出 `ODOM_STATE` 二进制帧：

- 帧长：45 字节
- 频率：200 Hz
- 帧头：`AA 55`
- 类型：`0x02`
- payload：`<QffffffHBB`
- CRC：CRC-16/CCITT，范围为 version 到 payload 末尾

上位机命令仍走同一 UART：

- `0x20 TIME_SYNC_REQ`：回 `0x21 TIME_SYNC_RESP`
- `0x30 SET_LOCAL_ORIGIN`：回 `0x31 SET_LOCAL_ORIGIN_ACK`

命令响应在 main loop 中处理，不在 TIM11 ISR 中执行。响应发送使用独立 DMA 缓冲，并通过三态保护阻止 200 Hz 状态帧抢占 ACK，直到响应 DMA 完成。

## SET_LOCAL_ORIGIN 行为

payload 为 `float x, y, yaw + uint8 flags + 3 reserved`。

flags：

- bit0：reset xy
- bit1：reset yaw
- bit2：reset encoder
- flags 为 0 时按 reset all 兼容旧上位机

执行效果：

- reset xy：覆盖 `X_tt/Y_tt/REAL_X/REAL_Y`
- reset yaw：设置本地 yaw 原点，并重置连续 yaw unwrap
- reset encoder：清零 AS5048 累计角度、delta、圈数，并把 last_angle 锚到当前角度
- 每次归零都会清空运行时速度积分缓存

## 有效性与降级

YIS130 的 gyro/euler CAN 帧有 100 ms freshness 检查。CAN 断流或 YIS130 停止输出时，固件会清掉 `IMU_VALID/YAW_VALID/POS_VALID/VEL_VALID`，并设置 `DEGRADED`。

AS5048 任一通道 invalid 时，固件不会积分该次编码器 delta，也不会继续声明位置/速度有效。

## 后续接正交轮时必须实测

- SPI1/SPI2 是否确实对应 X/Y tracking wheel
- `ODOM_X_ENCODER_SIGN`、`ODOM_Y_ENCODER_SIGN` 方向
- `ODOM_X_WHEEL_Y_OFFSET_M`、`ODOM_Y_WHEEL_X_OFFSET_M` 旋转补偿偏置
- `WHEEL_DIAMETER_M` 和每圈实际行程标定
- AS5048 `valid/error_count/last_error_flags` 在实际线束上的稳定性

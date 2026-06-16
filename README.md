# STM32底盘里程计固件

## 硬件配置

### 传感器
- **AS5048A磁编码器 x2**（SPI）
  - 编码器0 (SPI1): X轴/前进方向
  - 编码器1 (SPI2): Y轴/左侧方向
  - 分辨率: 14位 (16384 counts/rev)
  - 采样频率: 20kHz

- **YIS130 IMU**（CAN总线）
  - 加速度计、陀螺仪、欧拉角
  - 更新频率: 100Hz
  - 用于yaw角融合

### 主控
- **STM32F407VET6**
  - 168MHz ARM Cortex-M4
  - 串口输出: UART1 @ 115200

## 固件功能

### 数据融合
- 20kHz高频采样编码器增量
- 实时yaw角补偿（body → world坐标变换）
- 200Hz输出频率
- 精度: 位置<0.1mm, yaw<0.02°

### 输出协议
二进制ODOM_STATE帧（45字节/帧）：
```
[AA 55] [Ver] [Type] [Seq] [Len] [Payload 36B] [CRC16]
```

Payload结构：
```c
uint64_t t_sample_us;  // 时间戳
float x, y;            // 位置 (m)
float yaw;             // 航向角 (rad)
float vx, vy, wz;      // 速度 (m/s, rad/s)
uint16_t status_bits;  // 传感器状态
uint8_t quality;       // 质量等级 0-3
```

## 编译与烧录

### 环境
- Keil MDK-ARM
- STM32CubeMX 6.x
- STLink烧录器

### 步骤
1. 打开项目: `Prototype_04_407_VE_Matlab/type_04_407VE/MDK-ARM/type_04_407VE.uvprojx`
2. 编译: Project → Build Target (F7)
3. 烧录: Flash → Download (F8)

## 测试

### Python测试脚本
```bash
# 融合测试（60秒）
python3 test_fusion.py --port /dev/ttyUSB0 --duration 60

# 稳定性测试（5分钟）
python3 test_stability.py --port /dev/ttyUSB0 --duration 5

# 单元测试
python3 test_zero_detection.py
```

### 预期性能
- 编码器有效率: >99%
- 位置噪声: σ<0.03mm
- Yaw噪声: σ<0.02°
- 速度噪声: <0.03 m/s（静止）

## 重要修复

### AS5048 SPI通信优化（2026-06-15）
**问题**: SPI2偶发读到0x0000导致±117mm位置跳变

**修复**:
1. 智能全0检测（物理可行性判断）
2. SPI2降速至2.625MHz
3. last_angle锚定防止delta累积

**效果**: 位置跳变消除99.88%，有效率从60%提升至100%

详见Git提交: `f200fd7`

## 文件说明

```
MDK-ARM/
├── AS5048.c/h          # 编码器驱动
├── YIS130.c/h          # IMU驱动
└── odom_protocol.h     # 协议定义

Core/Src/
├── main.c              # 主程序（融合算法）
└── spi.c               # SPI配置

tools/
└── odom_parser.py      # Python解析库

test_fusion.py          # 融合测试
test_stability.py       # 稳定性测试
test_zero_detection.py  # 单元测试
```

## 坐标系定义

- **Body Frame**: 机器人本体坐标系
  - X轴: 前进方向
  - Y轴: 左侧方向
  - Z轴: 向上（右手系）

- **World Frame**: 固定世界坐标系
  - 通过yaw角旋转body frame得到
  - 初始对齐body frame

## 配置参数

位于 `MDK-ARM/YIS130.h`:
```c
#define ODOM_X_ENCODER_SIGN 1.0f        // X轮方向
#define ODOM_Y_ENCODER_SIGN 1.0f        // Y轮方向
#define ODOM_X_WHEEL_Y_OFFSET_M 0.0f    // X轮Y偏移
#define ODOM_Y_WHEEL_X_OFFSET_M 0.0f    // Y轮X偏移
```

## 故障排查

### 串口无数据
- 检查波特率115200
- 检查UART1引脚：PA9(TX), PA10(RX)

### 位置跳变
- 确认AS5048磁铁安装牢固
- 检查SPI线缆长度<20cm
- 验证test_zero_detection.py通过

### Yaw角跳变
- 检查CAN总线终端电阻
- 确认YIS130供电稳定5V

# 编码器方向修复说明

## 问题
右轮编码器（X轴，SPI1）方向反向，导致：
- 向前推 → 向左前方移动
- 向后推 → 向右后方移动

## 诊断结果
```
左轮向前: body角度 = +90.0° ✅ 正确
右轮向前: body角度 = +180.0° ❌ 应为0°，反向了
```

## 修复
修改 `MDK-ARM/YIS130.h`:
```c
#define ODOM_X_ENCODER_SIGN -1.0f   // 从 1.0f 改为 -1.0f
#define ODOM_Y_ENCODER_SIGN 1.0f    // 保持不变
```

## 重新编译烧录
1. 打开Keil项目: `MDK-ARM/type_04_407VE.uvprojx`
2. 编译: F7
3. 烧录: F8
4. 重启MCU

## 验证
烧录后重新测试：
```bash
python3 diagnose_encoders.py
```

预期结果：
- 右轮向前 → body角度 = 0° ✅
- 左轮向前 → body角度 = 90° ✅
- 机器人向前推 → 只有X增加，Y不变

## Git提交
```bash
git add MDK-ARM/YIS130.h
git commit -m "fix: reverse X encoder sign to match hardware orientation"
```

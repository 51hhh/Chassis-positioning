#!/usr/bin/env python3
"""
编码器方向诊断工具

测试步骤：
1. 机器人静止，记录初始位置
2. **只转动X轮**（编码器0，SPI1），观察world坐标变化
3. **只转动Y轮**（编码器1，SPI2），观察world坐标变化
4. 根据结果给出修复建议
"""

import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class EncoderDiagnostic(Node):
    def __init__(self):
        super().__init__('encoder_diagnostic')
        self.sub = self.create_subscription(Odometry, '/mcu_odom', self.callback, 10)
        self.x0, self.y0, self.yaw0 = None, None, None

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = math.atan2(2*qw*qz, 1-2*qz*qz)
        yaw_deg = yaw_rad * 180 / math.pi

        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = x, y, yaw_rad
            print(f"\n初始位置: x={x:.4f}m, y={y:.4f}m, yaw={yaw_deg:.2f}°")
            print("\n" + "="*70)
            print("测试步骤:")
            print("1. 手动转动X轮编码器（前轮，SPI1）正向约1圈，然后按Ctrl+C")
            print("2. 记录world坐标变化")
            print("3. 重启脚本，转动Y轮编码器（侧轮，SPI2）正向约1圈")
            print("="*70 + "\n")
        else:
            dx_world = x - self.x0
            dy_world = y - self.y0
            dyaw = yaw_rad - self.yaw0

            # 反向旋转到body frame
            cos_yaw = math.cos(-self.yaw0)
            sin_yaw = math.sin(-self.yaw0)
            dx_body = dx_world * cos_yaw - dy_world * sin_yaw
            dy_body = dx_world * sin_yaw + dy_world * cos_yaw

            dist = math.sqrt(dx_world**2 + dy_world**2)
            angle_world = math.atan2(dy_world, dx_world) * 180 / math.pi
            angle_body = math.atan2(dy_body, dx_body) * 180 / math.pi

            print(f"\r位移: world({dx_world:+.4f}, {dy_world:+.4f})={dist:.4f}m@{angle_world:+.1f}° | "
                  f"body({dx_body:+.4f}, {dy_body:+.4f})@{angle_body:+.1f}°", end='')

def main():
    print("\n" + "="*70)
    print("编码器方向诊断工具")
    print("="*70)

    rclpy.init()
    node = EncoderDiagnostic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("诊断结果分析:")
        print("="*70)
        if node.x0 is not None:
            print("\n请根据body frame角度判断:")
            print("  - 转X轮 → body角度应为  0°（正向）或 180°（反向）")
            print("  - 转Y轮 → body角度应为 90°（正向）或 -90°（反向）")
            print("\n如果角度偏差>45°，说明编码器定义错误，需要修改固件配置。")
        print("="*70 + "\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

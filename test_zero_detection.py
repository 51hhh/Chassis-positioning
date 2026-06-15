#!/usr/bin/env python3
"""
验证AS5048全0检测逻辑的正确性

测试场景：
1. 真实角度为0°时不应被拒绝
2. 从远离0°跳到0°应被拒绝（SPI故障）
"""

def test_zero_detection_logic():
    """模拟AS5048_Read中的全0检测逻辑"""
    HALF_COUNTS = 8192
    FULL_COUNTS = 16384
    THRESHOLD = 100  # 距离0°的阈值

    test_cases = [
        # (response, current_angle, expected_valid, description)
        (0x0000, 0, True, "真实0° - 已经在0°"),
        (0x0000, 1, True, "真实0° - 从1平滑过渡"),
        (0x0000, 50, True, "真实0° - 从50渐变 (<100阈值)"),
        (0x0000, 99, True, "真实0° - 边界情况 (99<100)"),
        (0x0000, 100, True, "真实0° - 刚好在阈值"),
        (0x0000, 101, False, "SPI故障 - 从101跳到0 (>100)"),
        (0x0000, 500, False, "SPI故障 - 从500跳到0"),
        (0x0000, 7448, False, "SPI故障 - 从7448跳到0 (实测场景)"),
        (0x0000, 16383, True, "真实0° - 从16383绕过0°"),
        (0x0000, 16300, True, "真实0° - 从16300渐变 (距0°=84<100)"),
        (0x0000, 16200, False, "SPI故障 - 从16200跳变 (距0°=184>100)"),
        (0x1234, 7448, True, "正常读取 - 非0响应始终接受"),
    ]

    print("=" * 80)
    print("AS5048 全0检测逻辑验证")
    print("=" * 80)
    print(f"阈值: {THRESHOLD} counts (~0.6°)")
    print()

    passed = 0
    failed = 0

    for response, angle, expected_valid, desc in test_cases:
        # 模拟固件逻辑
        if response == 0x0000 and angle != 0:
            dist = angle if angle < HALF_COUNTS else (FULL_COUNTS - angle)
            actual_valid = (dist <= THRESHOLD)
        else:
            actual_valid = True

        status = "✓" if actual_valid == expected_valid else "✗"
        result = "PASS" if actual_valid == expected_valid else "FAIL"

        if actual_valid == expected_valid:
            passed += 1
        else:
            failed += 1

        print(f"{status} {result:4s} | resp=0x{response:04X} angle={angle:5d} "
              f"→ valid={actual_valid:5} | {desc}")

    print()
    print("=" * 80)
    print(f"测试结果: {passed} passed, {failed} failed")
    print("=" * 80)

    return failed == 0

if __name__ == "__main__":
    success = test_zero_detection_logic()
    exit(0 if success else 1)

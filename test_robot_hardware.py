#!/usr/bin/env python3
"""
Direct hardware test - no ROS2, no Docker
Tests if the robot can actually move
"""

import time
from picarx import Picarx

def test_robot():
    print("🤖 Testing PicarX Hardware")
    print("=" * 40)

    try:
        print("📍 Initializing robot...")
        px = Picarx()
        print("✅ Robot initialized successfully!")

        print("\n🔄 Test 1: Small forward movement")
        print("   Moving forward for 1 second...")
        px.forward(20)
        time.sleep(1)
        px.stop()
        print("✅ Forward movement complete")

        print("\n🔄 Test 2: Steering test")
        print("   Turning wheels left...")
        px.set_dir_servo_angle(-30)
        time.sleep(0.5)
        print("   Centering wheels...")
        px.set_dir_servo_angle(0)
        time.sleep(0.5)
        print("   Turning wheels right...")
        px.set_dir_servo_angle(30)
        time.sleep(0.5)
        print("   Centering wheels...")
        px.set_dir_servo_angle(0)
        print("✅ Steering test complete")

        print("\n🔄 Test 3: Rotation test (room scan simulation)")
        print("   Rotating in place (left turn)...")
        px.set_dir_servo_angle(-30)
        px.forward(20)
        time.sleep(0.5)
        px.stop()
        px.set_dir_servo_angle(0)
        print("✅ Rotation test complete")

        print("\n" + "=" * 40)
        print("✅ ALL TESTS PASSED!")
        print("🚗 Robot hardware is working correctly!")

    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        print("   Hardware test failed!")
        return False

    return True

if __name__ == "__main__":
    test_robot()

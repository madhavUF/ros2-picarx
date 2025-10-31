#!/usr/bin/env python3
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from robot_hat import ADC, Pin
import time

class MultiSensorPublisher(Node):
    def __init__(self):
        super().__init__('multi_sensor_publisher')
        self.distance_pub = self.create_publisher(Float32, 'sensor/distance', 10)
        self.greyscale_pub = self.create_publisher(String, 'sensor/greyscale', 10)

        # Ultrasonic setup - remove active_state parameter that's causing issues
        self.trig = Pin("D2", mode=Pin.OUT)
        self.echo = Pin("D3", mode=Pin.IN)

        # Grayscale sensors - remove active_state parameter
        self.greyscale_pins = [
            Pin("D0", mode=Pin.IN),
            Pin("D1", mode=Pin.IN),
            Pin("D4", mode=Pin.IN)
        ]

        self.timer = self.create_timer(1.0, self.publish_sensors)
        self.get_logger().info("📡 MultiSensorPublisher node started")

    def measure_distance(self):
        self.trig.value(0)
        time.sleep(0.002)
        self.trig.value(1)
        time.sleep(0.01)
        self.trig.value(0)

        timeout = time.time() + 0.05
        while self.echo.value() == 0:
            if time.time() > timeout:
                return -1
        start = time.time()

        timeout = time.time() + 0.05
        while self.echo.value() == 1:
            if time.time() > timeout:
                return -1
        end = time.time()

        duration = end - start
        distance_cm = round(duration * 17150, 2)
        return distance_cm

    def read_greyscale(self):
        values = [str(pin.value()) for pin in self.greyscale_pins]
        return ','.join(values)

    def publish_sensors(self):
        distance = self.measure_distance()
        greyscale = self.read_greyscale()

        distance_msg = Float32()
        distance_msg.data = float(distance)
        self.distance_pub.publish(distance_msg)

        grey_msg = String()
        grey_msg.data = greyscale
        self.greyscale_pub.publish(grey_msg)

        self.get_logger().info(f"📏 Distance: {distance} cm | 🎨 Grayscale: {greyscale}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

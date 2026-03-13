import rclpy
from rclpy.node import Node
from robot_hat import ADC, Servo, Pin
import time

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')

        self.trig = Pin("D2")
        self.echo = Pin("D3", mode=Pin.IN, active_state=True)
        self.adc = ADC(0)

        self.timer = self.create_timer(1.0, self.read_sensors)
        self.get_logger().info('📡 SensorReader node started')

    def read_sensors(self):
        voltage = round(self.adc.read_voltage(), 2)
        distance = self.measure_distance()
        self.get_logger().info(f'🔋 Voltage: {voltage} V | 📏 Distance: {distance} cm')

    def measure_distance(self):
        self.trig.value(0)
        time.sleep(0.002)
        self.trig.value(1)
        time.sleep(0.01)
        self.trig.value(0)

        timeout = time.time() + 0.05  # 50 ms timeout
        while self.echo.value() == 0:
            if time.time() > timeout:
                return -1  # timeout
        start = time.time()

        timeout = time.time() + 0.05
        while self.echo.value() == 1:
            if time.time() > timeout:
                return -1
        end = time.time()

        duration = end - start
        distance_cm = round(duration * 17150, 2)
        return distance_cm

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

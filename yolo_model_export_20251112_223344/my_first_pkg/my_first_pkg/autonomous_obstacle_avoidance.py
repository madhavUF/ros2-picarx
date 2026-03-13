import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time
import random

class AutonomousObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('autonomous_obstacle_avoidance')
        
        # Subscribers for sensor data
        self.distance_subscription = self.create_subscription(
            Float32,
            'sensor/distance',
            self.distance_callback,
            10
        )
        
        # Publisher for driving commands
        self.drive_publisher = self.create_publisher(String, 'cmd_drive', 10)
        
        # Robot state variables
        self.current_distance = 999.0  # Start with large distance
        self.obstacle_threshold = 30.0  # Stop when object within 30cm
        self.turn_threshold = 50.0      # Start turning when object within 50cm
        
        # State machine for robot behavior
        self.robot_state = "FORWARD"  # FORWARD, TURNING, BACKING_UP
        self.turn_direction = "right"  # right or left
        self.turn_start_time = None
        self.turn_duration = 2.0  # Turn for 2 seconds
        
        # Control timer - make decisions every 100ms
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🤖 Autonomous Obstacle Avoidance started!')
        self.get_logger().info(f'📏 Obstacle threshold: {self.obstacle_threshold}cm')
        self.get_logger().info(f'⚠️ Turn threshold: {self.turn_threshold}cm')

    def distance_callback(self, msg):
        """Receive distance sensor data"""
        self.current_distance = msg.data
        
    def control_loop(self):
        """Main control logic - runs every 100ms"""
        
        # Skip if we don't have sensor data yet
        if self.current_distance == 999.0:
            return
            
        # Treat -1 (timeout) as clear path - no obstacle detected
        effective_distance = self.current_distance
        if self.current_distance < 0:
            effective_distance = 200.0  # Treat timeout as clear path (200cm)
            
        # State machine for obstacle avoidance
        if self.robot_state == "FORWARD":
            self.handle_forward_state(effective_distance)
        elif self.robot_state == "TURNING":
            self.handle_turning_state(effective_distance)
        elif self.robot_state == "BACKING_UP":
            self.handle_backing_state(effective_distance)
    
    def handle_forward_state(self, distance):
        """Handle forward movement and obstacle detection"""
        if distance < self.obstacle_threshold:
            # Very close obstacle - back up first
            self.get_logger().info(f'🚨 OBSTACLE DETECTED at {distance:.1f}cm - BACKING UP')
            self.send_command("backward")
            self.robot_state = "BACKING_UP"
            self.turn_start_time = time.time()
            
        elif distance < self.turn_threshold:
            # Obstacle ahead - start turning
            self.turn_direction = random.choice(["left", "right"])  # Random turn direction
            self.get_logger().info(f'⚠️ Obstacle at {distance:.1f}cm - TURNING {self.turn_direction.upper()}')
            self.send_command(self.turn_direction)
            self.robot_state = "TURNING"
            self.turn_start_time = time.time()
            
        else:
            # Clear path - move forward
            self.send_command("forward")
            # Log status less frequently
            if int(time.time()) % 5 == 0:  # Log every 5 seconds
                status = "timeout (clear)" if self.current_distance < 0 else f"{distance:.1f}cm"
                self.get_logger().info(f'✅ Moving forward - distance: {status}')
    
    def handle_turning_state(self, distance):
        """Handle turning to avoid obstacle"""
        if time.time() - self.turn_start_time > self.turn_duration:
            # Finished turning
            if distance > self.turn_threshold:
                # Path is clear - resume forward
                self.get_logger().info(f'✅ Turn complete - resuming forward (distance: {distance:.1f}cm)')
                self.robot_state = "FORWARD"
            else:
                # Still blocked - turn some more
                self.get_logger().info(f'🔄 Still blocked - continuing turn {self.turn_direction}')
                self.turn_start_time = time.time()
    
    def handle_backing_state(self, distance):
        """Handle backing up from very close obstacle"""
        if time.time() - self.turn_start_time > 1.0:  # Back up for 1 second
            # Switch to turning
            self.turn_direction = random.choice(["left", "right"])
            self.get_logger().info(f'🔄 Backup complete - now turning {self.turn_direction.upper()}')
            self.send_command(self.turn_direction)
            self.robot_state = "TURNING"
            self.turn_start_time = time.time()
    
    def send_command(self, command):
        """Send driving command to the robot"""
        msg = String()
        msg.data = command
        self.drive_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Stopping autonomous navigation')
    finally:
        # Stop the robot
        stop_msg = String()
        stop_msg.data = "stop"
        node.drive_publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

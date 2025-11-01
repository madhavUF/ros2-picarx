#!/usr/bin/env python3
"""
Person Tracker Node

Subscribes to: /vision/detections (DetectionArray)
Publishes:
  - /tracking/target_person (TargetPerson): Information about tracked person
  - /tracking/person_state (String): Current tracking state (found/lost/scanning)
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import DetectionArray, TargetPerson
from std_msgs.msg import String


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')

        # Parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('min_area', 5000.0)
        self.declare_parameter('lost_threshold', 10)  # frames

        self.min_conf = self.get_parameter('min_confidence').value
        self.min_area = self.get_parameter('min_area').value
        self.lost_threshold = self.get_parameter('lost_threshold').value

        # State
        self.frames_without_person = 0

        # Subscriber
        self.detection_sub = self.create_subscription(
            DetectionArray, '/vision/detections', self.detection_callback, 10)

        # Publishers
        self.target_pub = self.create_publisher(
            TargetPerson, '/tracking/target_person', 10)
        self.state_pub = self.create_publisher(
            String, '/tracking/person_state', 10)

        self.get_logger().info(
            f'Person Tracker Node started '
            f'(min_conf={self.min_conf}, min_area={self.min_area})')

    def detection_callback(self, msg):
        # Filter for persons
        persons = [d for d in msg.detections
                   if d.class_name == 'person'
                   and d.confidence >= self.min_conf
                   and d.area >= self.min_area]

        target = TargetPerson()
        target.header = msg.header

        if persons:
            # Select best person (highest confidence)
            best_person = max(persons, key=lambda p: p.confidence)

            target.person_detected = True
            target.confidence = best_person.confidence
            target.center_x = best_person.center_x
            target.center_y = best_person.center_y
            target.area = best_person.area

            # Calculate horizontal offset from center
            frame_center = msg.frame_width / 2
            target.horizontal_offset = best_person.center_x - frame_center

            # Estimate distance based on area
            target.distance_estimate = self.estimate_distance(best_person.area)

            # Reset lost counter
            self.frames_without_person = 0

            # Publish state
            state_msg = String()
            state_msg.data = 'found'
            self.state_pub.publish(state_msg)

            self.get_logger().info(
                f'Person tracked: conf={target.confidence:.2f}, '
                f'area={target.area:.0f}, offset={target.horizontal_offset:.1f}px',
                throttle_duration_sec=1.0)
        else:
            target.person_detected = False
            self.frames_without_person += 1

            # Publish state
            state_msg = String()
            if self.frames_without_person < self.lost_threshold:
                state_msg.data = 'lost'
            else:
                state_msg.data = 'scanning'
            self.state_pub.publish(state_msg)

            if self.frames_without_person == self.lost_threshold:
                self.get_logger().warn('Person lost, initiating scan mode')

        # Always publish target (even if not detected)
        self.target_pub.publish(target)

    def estimate_distance(self, area):
        """Estimate distance category based on bounding box area"""
        if area > 150000:
            return 1.0  # too close
        elif area > 50000:
            return 2.0  # good distance
        elif area > 15000:
            return 3.0  # medium distance
        else:
            return 4.0  # far away


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

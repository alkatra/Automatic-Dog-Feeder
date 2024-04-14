import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import random


class MotionSensorNode(Node):
    """
    A ROS2 node that simulates a motion sensor and publishes the motion status.

    This node periodically checks the motion sensor state and publishes the motion status
    as a boolean value. It simulates motion detection with a 10% chance of detecting motion,
    or if motion was detected in the last 10 seconds to simulate a dog in front.

    Attributes:
        publisher_: A publisher for publishing the motion status.
        time_since_start: The time elapsed since the node started.
        timer_period: The period of the timer for checking the motion sensor state.
        timer: A timer for periodically checking the motion sensor state.
        detected_seconds: The number of seconds since motion was last detected.
    """

    def __init__(self):
        """
        Initialize the MotionSensorNode.
        """
        super().__init__('motion_sensor_node')
        self.publisher_ = self.create_publisher(Bool, 'sensors/motion', 10)
        self.time_since_start = 0
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.check_motion)
        self.detected_seconds = 0

        self.get_logger().info('MotionSensorNode has started.')

    def check_motion(self):
        """
        Check the motion sensor state and publish the motion status.

        This method is called periodically by the timer. It simulates motion detection
        with a 10% chance of detecting motion, or if motion was detected in the last
        10 seconds to simulate a dog in front. If motion is detected, the motion status
        is published as True. Otherwise, it is published as False.
        """
        self.time_since_start += 1
        if random.randint(0, 100) < 10 or (self.detected_seconds > 0 and self.detected_seconds <= 10):
            motion_detected = True
            self.detected_seconds += 1
            if self.detected_seconds >= 10:
                self.detected_seconds = 0
        else:
            motion_detected = False
        
        self.publish_motion_status(motion_detected)

    def publish_motion_status(self, motion_detected):
        """
        Publish the motion status.

        Args:
            motion_detected: A boolean value indicating whether motion is detected.
        """
        motion_message = Bool()
        motion_message.data = motion_detected
        self.publisher_.publish(motion_message)
        self.get_logger().info('AUTOMATIC DOG FEEDER - RUNNING FOR %d SECONDS' % self.time_since_start)
        if motion_detected:
            self.get_logger().info('Motion Detected: True')
        else:
            self.get_logger().info('Motion Detected: False')

def main(args=None):
    rclpy.init(args=args)
    motion_sensor_node = MotionSensorNode()
    try:
        rclpy.spin(motion_sensor_node)
    finally:
        motion_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

class MotionSensorNode(Node):
    def __init__(self):
        super().__init__('motion_sensor_node')
        self.publisher_ = self.create_publisher(Bool, 'motion_status', 10)
        
        # GPIO setup
        self.pir_pin = 9 # Assumes PIR sensor is connected to GPIO 9
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pir_pin, GPIO.IN)
        
        # Timer to periodically check the PIR sensor state
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.check_motion)

        self.get_logger().info('MotionSensorNode has started.')

    def check_motion(self):
        if GPIO.input(self.pir_pin):
            motion_detected = True
        else:
            motion_detected = False
        
        self.publish_motion_status(motion_detected)

    def publish_motion_status(self, motion_detected):
        motion_message = Bool()
        motion_message.data = motion_detected
        self.publisher_.publish(motion_message)
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
        # Cleanup GPIO when the node is stopped
        GPIO.cleanup()
        motion_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

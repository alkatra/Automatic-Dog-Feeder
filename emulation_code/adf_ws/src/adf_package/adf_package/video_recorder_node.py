import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from datetime import datetime
import threading


class VideoRecorderNode(Node):
    """
    A ROS2 node that records video when motion is detected.

    This node subscribes to the 'motion_status' topic and starts recording when motion is detected.
    It stops recording when motion stops.

    Attributes:
        subscription: A subscription object for the 'motion_status' topic.
        is_recording: A boolean indicating whether the node is currently recording.
        filename: A string representing the filename of the recorded video.
    """

    def __init__(self):
        super().__init__('video_recorder_node')
        self.subscription = self.create_subscription(
            Bool,
            'sensors/motion',
            self.motion_callback,
            10)
        self.is_recording = False
        self.get_logger().info('VideoRecorderNode has started.')

    def motion_callback(self, msg):
        """
        Callback function for the 'motion_status' topic.

        Args:
            msg: A std_msgs/Bool message indicating the motion status.
        """
        if msg.data and not self.is_recording:
            self.start_recording()
        elif not msg.data and self.is_recording:
            self.stop_recording()

    def start_recording(self):
        """
        Starts the video recording.

        This method is called when motion is detected and recording is not already in progress.
        It sets the 'is_recording' flag to True and generates a filename based on the current timestamp.
        """
        self.get_logger().info('Motion detected, starting recording...')
        self.filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S.h264')
        self.is_recording = True

    def stop_recording(self):
        """
        Stops the video recording.

        This method is called when motion stops and recording is in progress.
        It sets the 'is_recording' flag to False and logs the filename of the recorded video.
        """
        self.get_logger().info('Motion stopped, recording saved to filename ' + self.filename)
        self.is_recording = False

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

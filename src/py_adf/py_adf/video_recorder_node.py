import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from picamera import PiCamera
from datetime import datetime
import threading

class CameraMotionNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        self.subscription = self.create_subscription(
            Bool,
            'motion_status',
            self.motion_callback,
            10)
        # Assumes the camera is connected to the Raspberry Pi's CSI port
        self.camera = PiCamera()
        self.is_recording = False
        # Lock to prevent multiple threads from accessing the camera at the same time
        self.recording_lock = threading.Lock()

    def motion_callback(self, msg):
        with self.recording_lock:
            if msg.data and not self.is_recording:
                self.start_recording()
            elif not msg.data and self.is_recording:
                self.stop_recording()

    def start_recording(self):
        self.get_logger().info('Motion detected, starting recording...')
        filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S.h264')
        self.camera.start_recording(filename)
        self.is_recording = True

    def stop_recording(self):
        self.camera.stop_recording()
        self.get_logger().info('Motion stopped, recording saved.')
        self.is_recording = False

def main(args=None):
    rclpy.init(args=args)
    node = CameraMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.camera.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import time


class DispenserControlNode(Node):
    """
    Node class for controlling the dispenser actuation.

    This class creates subscribers for motion status, food bowl weight, and water bowl weight.
    It also creates publishers for food actuator status and water actuator status.
    The class implements callback functions for the subscribers to check the bowl weights and dispense food or water if necessary.
    """

    def __init__(self):
        super().__init__('actuation_node')
        self.motion_subscriber = self.create_subscription(
            Bool,
            'sensors/motion',
            self.motion_callback,
            10)
        self.food_bowl_weight_subscriber = self.create_subscription(
            Int32,
            'sensors/foodweight',
            self.food_bowl_weight_callback,
            10)
        self.water_bowl_weight_subscriber = self.create_subscription(
            Int32,
            'sensors/waterweight',
            self.water_bowl_weight_callback,
            10)
        
        self.food_actuator_publisher = self.create_publisher(Bool, 'actuators/food', 10)
        self.water_actuator_publisher = self.create_publisher(Bool, 'actuators/water', 10)
        
        self.food_bowl_weight = 0
        self.water_bowl_weight = 0
          
        self.food_threshold = 50  # Assumes threshold weight of 50 grams per bowl
        self.water_threshold = 120 # Assumes threshold weight of 120 grams per bowl  

        self.get_logger().info('DispenserControlNode started')

    def food_bowl_weight_callback(self, msg):
        self.food_bowl_weight = msg.data

    def water_bowl_weight_callback(self, msg):
        self.water_bowl_weight = msg.data

    def motion_callback(self, msg):
        if msg.data:  # Motion detected
            if self.food_bowl_weight < self.food_threshold:
                # Dispense food
                self.get_logger().info('Dispensing food...')
                self.food_actuator_publisher.publish(Bool(data=True))
            if self.water_bowl_weight < self.water_threshold:
                # Dispense water
                self.get_logger().info('Dispensing water...')
                self.water_actuator_publisher.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = DispenserControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

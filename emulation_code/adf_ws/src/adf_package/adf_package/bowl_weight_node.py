import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import random


class BowlWeightNode(Node):
    """
    This class represents a ROS2 node that publishes the weight of food and water bowls.
    It emulates the increment/decrement of weight based on actuator status and motion status.
    """

    def __init__(self):
        super().__init__('bowl_weight_node')
        
        # Initial weights
        self.current_food_weight = 120
        self.current_water_weight = 130

        # EMULATION PURPOSES ONLY
        # Subscribe to food and water bowl actuator status and motion status
        # to emulate increment/decrement of weight
        self.food_actuator_subscriber = self.create_subscription(Bool, 'actuators/food', self.food_actuator_callback, 10)
        self.water_actuator_subscriber = self.create_subscription(Bool, 'actuators/water', self.water_actuator_callback, 10)
        self.motion_status_subscriber = self.create_subscription(Bool, 'sensors/motion', self.motion_status_callback, 10)

        self.food_bowl_weight_publisher_ = self.create_publisher(Int32, 'sensors/foodweight', 10)
        self.water_bowl_weight_publisher_ = self.create_publisher(Int32, 'sensors/waterweight', 10)

        # Timer to periodically publish the weight of the food and water bowls
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.weight_publisher)

        self.get_logger().info('BowlWeightPublisher Node started')

    def food_actuator_callback(self, msg):
        """
        Callback function for the food actuator status subscriber.
        Emulates the increment/decrement of food weight based on the actuator status.
        """
        if msg.data:
            self.current_food_weight += 25
        else:
            self.current_food_weight -= 25

    def water_actuator_callback(self, msg):
        """
        Callback function for the water actuator status subscriber.
        Emulates the increment/decrement of water weight based on the actuator status.
        """
        if msg.data:
            self.current_water_weight += 25
        else:
            self.current_water_weight -= 25

    def motion_status_callback(self, msg):
        """
        Callback function for the motion status subscriber.
        Emulates the decrement of food and water weight based on the motion status.
        """
        if msg.data:
            self.current_food_weight -= 20
            self.current_water_weight -= 20

    def weight_publisher(self):
        """
        Publishes the current weight of the food and water bowls.
        """
        food_weight_msg = Int32()
        water_weight_msg = Int32()
        food_weight_msg.data = self.current_food_weight
        water_weight_msg.data = self.current_water_weight
        self.food_bowl_weight_publisher_.publish(food_weight_msg)
        self.water_bowl_weight_publisher_.publish(water_weight_msg)

        self.get_logger().info('Current Food Weight: %d' % self.current_food_weight)
        self.get_logger().info('Current Water Weight: %d' % self.current_water_weight)
        

def main(args=None):
    rclpy.init(args=args)
    node = BowlWeightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

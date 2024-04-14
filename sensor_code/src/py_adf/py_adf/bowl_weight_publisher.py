import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from hx711 import HX711
from adf_py.srv import GetBowlWeights

class BowlWeightPublisher(Node):
    def __init__(self):
        super().__init__('bowl_weight_publisher')
        
        # Service to get current weights
        self.service = self.create_service(GetBowlWeights, 'get_bowl_weights', self.handle_get_weights_request)
        
        # HX711 setup for food and water bowls
        self.food_hx711 = HX711(dout_pin=5, pd_sck_pin=6)  # Assumes food weight sensor is connected to GPIO 5 and 6
        self.water_hx711 = HX711(dout_pin=7, pd_sck_pin=8)  # Assumes water weight sensor is connected to GPIO 7 and 8
        
        # Initial calibration
        self.food_hx711.reset()  # Resets the HX711
        self.food_hx711.tare()  # Resets the weight to 0, assuming no weight is on the sensor
        
        self.water_hx711.reset()
        self.water_hx711.tare()
        
        self.get_logger().info('BowlWeightPublisher Node started')

    def handle_get_weights_request(self, request, response):
        # Get the current weights from the sensors
        current_food_weight = self.food_hx711.get_weight_mean(5)  # Average of 5 readings
        current_water_weight = self.water_hx711.get_weight_mean(5)  
    
        response.weights = [current_food_weight, current_water_weight]  # Custom service response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BowlWeightPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

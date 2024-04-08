import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
from adf_py.srv import GetBowlWeights

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class DispenserControlNode(Node):
    def __init__(self):
        super().__init__('actuation_node')
        self.subscription = self.create_subscription(
            Bool,
            'motion_status',
            self.motion_callback,
            10)
        self.client = self.create_client(GetBowlWeights, 'get_bowl_weights')  
        self.food_servo_pin = 20  # Assumes food servo is connected to GPIO 20 (PWM pin)
        self.water_servo_pin = 21  # Assumes water servo is connected to GPIO 21 (PWM pin)
        GPIO.setup(self.food_servo_pin, GPIO.OUT)
        GPIO.setup(self.water_servo_pin, GPIO.OUT)
        self.food_servo = GPIO.PWM(self.food_servo_pin, 50)  # Set frequency to 50 Hz
        self.water_servo = GPIO.PWM(self.water_servo_pin, 50)  
        self.food_servo.start(0)  # Initialization
        self.water_servo.start(0)  
        self.food_threshold = 50  # Assumes threshold weight of 50 grams per bowl
        self.water_threshold = 120 # Assumes threshold weight of 120 grams per bowl  
        self.food_servo_open_duty_cycle = 7.5  # Assumes open position duty cycle of 7.5
        self.food_servo_close_duty_cycle = 2.5
        self.food_servo_dispense_time = 3  # Assumes dispensing time of 3 second
        self.water_servo_open_duty_cycle = 7.5
        self.water_servo_close_duty_cycle = 2.5
        self.water_servo_dispense_time = 2 # Assumes dispensing time of 2 second
        self.get_logger().info('DispenserControlNode started')

def motion_callback(self, msg):
    if msg.data:  # Motion detected
        self.get_logger().info('Motion detected, checking bowl weights...')
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Weight service not available...')
            return
        request = GetBowlWeights.Request()  # Create a request object
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            food_weight, water_weight = response.weights 
            if food_weight < self.food_threshold:
                # Dispense food
                self.food_servo.ChangeDutyCycle(self.food_servo_open_duty_cycle)
                time.sleep(self.food_servo_dispense_time)
                self.food_servo.ChangeDutyCycle(self.food_servo_close_duty_cycle)
            if water_weight < self.water_threshold:
                # Dispense water
                self.water_servo.ChangeDutyCycle(self.water_servo_open_duty_cycle)
                time.sleep(self.water_servo_dispense_time)
                self.water_servo.ChangeDutyCycle(self.water_servo_close_duty_cycle)

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
        GPIO.cleanup()

if __name__ == '__main__':
    main()

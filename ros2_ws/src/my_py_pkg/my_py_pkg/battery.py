#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_battery_interfaces.srv import SetLED
import time
from functools import partial

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery")
        self.client_ = self.create_client(SetLED, "set_led")
        self.battery_state = True # Battery state = full
        self.intervals = [4.0, 6.0]
        self.get_logger().info("Battery is full")
        self.call_set_led()

        while(True):
            time.sleep(self.intervals[0]) # Battery is full for 4 sec
            self.battery_state = False
            self.get_logger().info("Battery is empty")
            self.call_set_led()
            
            time.sleep(self.intervals[1]) # Battery is empty for 6 sec
            self.battery_state = True
            self.get_logger().info("Battery is full")
            self.call_set_led()

    def call_set_led(self):
        request = SetLED.Request()

        if(self.battery_state):    
            request.led_number = 3
            request.state = False
        else:
            request.led_number = 3
            request.state = True

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led, request=request))

    def callback_call_set_led(self, future, request):
        response = future.result()
        self.get_logger().info("Led number: " + str(request.led_number) + 
                                " ,state: " + str(request.state) + 
                                " ,Success: " + str(response.success))    

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
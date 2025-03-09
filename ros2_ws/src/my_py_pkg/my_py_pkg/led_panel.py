#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_battery_interfaces.srv import SetLED
from my_battery_interfaces.msg import LEDPanelState

class LEDPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_panel_state_ = self.get_parameter("led_states").value
        
        self.led_state_pub_ = self.create_publisher(
            LEDPanelState, "led_panel_state", 10)
        self.timer_ = self.create_timer(1, self.publish_state)

        self.server = self.create_service(
            SetLED, "set_led", self.callback_set_led)
        self.get_logger().info("LED Panel has been activated.")

    def callback_set_led(self, request: SetLED.Request, response: SetLED.Response):
        if(request.state):
            # battery is empty
            self.led_panel_state_[request.led_number-1] = 1
        else:
            # battery is full
            self.led_panel_state_[request.led_number-1] = 0

        response.success = True
        self.get_logger().info("Led number: " + str(request.led_number) + 
                                        " ,state: " + str(request.state) +
                                        " ,Success: " + str(response.success))
        return response

    def publish_state(self):
        msg = LEDPanelState()
        msg.led_state = self.led_panel_state_
        self.led_state_pub_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = LEDPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
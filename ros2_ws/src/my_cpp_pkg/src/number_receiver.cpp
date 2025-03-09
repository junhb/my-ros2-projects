#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include <inttypes.h>  // Include this for PRId64
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class NumberReceiverNode : public rclcpp::Node
{
public:
    NumberReceiverNode() : Node("number_counter")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        timer_ = this->create_wall_timer(0.5s, std::bind(&NumberReceiverNode::publishNumber, this));

        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberReceiverNode::callbackNumber, this, _1));
        
        // Activity 3 - Adding a service

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberReceiverNode::callbackSetBool, this, _1, _2));


        RCLCPP_INFO(this->get_logger(), "Number Receiver Activated");
    }
private:

    void publishNumber()
        {
            auto msg = example_interfaces::msg::Int64();
            msg.data = counter_; // Publish the updated counter value
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published counter: %" PRId64, counter_);
        }

    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
        {
            counter_ += msg->data; //Add received number to the counter
            RCLCPP_INFO(this->get_logger(), "Received: %" PRId64 ", Updated counter: %" PRId64, msg->data, counter_);
        }
    
    void callbackSetBool(const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Counter has been reset";
        }
        else
        {
            response->success = false;
            response->message = "Counter has not been reset";
        }
    }    
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        int64_t counter_; // Counter to accumulate received numbers

        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberReceiverNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

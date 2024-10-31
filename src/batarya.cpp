#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/set_led.hpp"

// 28 sept. edition

class BattNode : public rclcpp::Node
{
public:
    BattNode() : Node("battery_node"), battState("Full")
    {
        lastBattCheck = this->get_clock()->now().seconds();
        tmrObj = this->create_wall_timer(std::chrono::milliseconds(1500),
                                         std::bind(&BattNode::checkBattState, this));
        RCLCPP_INFO(this->get_logger(), "Battery node has been started...");
    }

private:
    double lastBattCheck;
    std::string battState;
    rclcpp::TimerBase::SharedPtr tmrObj;
    std::vector<std::shared_ptr<std::thread>> setLedThread;
    std::vector<std::thread> thrdObj;

    void setLED(int ledPos, int ledState)
        {thrdObj.push_back(std::thread(std::bind(&BattNode::setLedServiceClient, this, ledPos, ledState)));}

    void setLedServiceClient(int led_digit, int stt)
    {
        auto clientObj = this->create_client<custom_interfaces::srv::SetLED>("set_led");
        
        while (!clientObj->wait_for_service(std::chrono::seconds(1)))
            {RCLCPP_WARN(this->get_logger(), "Waiting for service to become available...");}

        auto reqObj = std::make_shared<custom_interfaces::srv::SetLED::Request>(); 
        reqObj->led_digit = led_digit;
        reqObj->state = stt;

        auto futurObj = clientObj->async_send_request(reqObj);

        try
        {
            auto respObj = futurObj.get();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "SERVICE CALL FAILED!!");
        }
    }

    void checkBattState()
    {
        double currTime = this->get_clock()->now().seconds();
        if (battState == "Full")
        {
            if (currTime - lastBattCheck > 4.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is depleted. Charging begins...");
                battState = "Empty";
                lastBattCheck = currTime;
                setLED(3, 1);
            }
        }
        else
        {
            if (currTime - lastBattCheck > 6.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is now completely charged. Drain begins...");
                battState = "Full";
                lastBattCheck = currTime;
                setLED(3, 0);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BattNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
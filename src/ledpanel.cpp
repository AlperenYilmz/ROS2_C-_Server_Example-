#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/led_states.hpp"
#include "custom_interfaces/srv/set_led.hpp"

class LedPanel : public rclcpp::Node
{
public:
    LedPanel() : Node("ledpanel_node"), ledsInitial(3, 0)
    {
        pubObj = this->create_publisher<custom_interfaces::msg::LedStates>("led_statuo", 10);
        taymir = this->create_wall_timer(std::chrono::seconds(2),
                                         std::bind(&LedPanel::pubberLedStates, this));
        ledSetterServiceObj = this->create_service<custom_interfaces::srv::SetLED>("setled",
                                                                                std::bind(&LedPanel::setLedServiceServiceBuild, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Led panel display just started...");
    }

private:
    void pubberLedStates()
    {
        auto mesaj = custom_interfaces::msg::LedStates();
        mesaj.leds_array = ledsInitial;
        pubObj->publish(mesaj);
    }

    void setLedServiceServiceBuild(const custom_interfaces::srv::SetLED::Request::SharedPtr REQU,
                      const custom_interfaces::srv::SetLED::Response::SharedPtr RESP)
    {
        int64_t lednumTemp = REQU->led_digit;
        int64_t statTemp = REQU->state;

        if (lednumTemp > (int64_t)ledsInitial.size() || lednumTemp <= 0)
        {
            RESP->success = false;
            return;
        }

        if (statTemp != 0 && statTemp != 1)
        {
            RESP->success = false;
            return;
        }

        ledsInitial.at(lednumTemp - 1) = statTemp;
        RESP->success = true;
        pubberLedStates();
    }
    
    rclcpp::Publisher<custom_interfaces::msg::LedStates>::SharedPtr pubObj;
    rclcpp::TimerBase::SharedPtr taymir;
    rclcpp::Service<custom_interfaces::srv::SetLED>::SharedPtr ledSetterServiceObj;
    std::vector<int64_t> ledsInitial;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

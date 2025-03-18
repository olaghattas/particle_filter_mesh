//
// Created by olagh48652 on 3/18/25.
//

#include <rclcpp/rclcpp.hpp>
#include <ctime>

class TimeCheckNode : public rclcpp::Node
{
public:
    TimeCheckNode() : Node("time_check_node")
    {
        timer_ = this->create_wall_timer(
                std::chrono::seconds(10),
                std::bind(&TimeCheckNode::check_time_window, this));
    }

private:
    void check_time_window()
    {
        std::time_t current_time = std::time(nullptr);
        std::tm *local_tm = std::localtime(&current_time);
        int hour = local_tm->tm_hour;  // 0-23

        if (hour >= 3 && hour < 7)
        {
            RCLCPP_INFO(this->get_logger(), "Current hour: %d — Within 3 AM to 7 AM", hour);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Current hour: %d — Outside 3 AM to 7 AM", hour);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimeCheckNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

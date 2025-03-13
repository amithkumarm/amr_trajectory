#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <random>
#include <chrono>

using namespace std::chrono_literals;

class TurtleBot3PathPlanner : public rclcpp::Node
{
public:
    TurtleBot3PathPlanner() : Node("turtlebot3_path_planner"), step_(0), rng_(rd_())
    {
        // Publisher for /cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Timer to update movement
        timer_ = this->create_wall_timer(500ms, std::bind(&TurtleBot3PathPlanner::move_robot, this));

        RCLCPP_INFO(this->get_logger(), "TurtleBot3 Path Planner Node Started");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
    std::random_device rd_;
    std::mt19937 rng_;
    std::uniform_int_distribution<int> random_action_{0, 3}; // 4 random actions

    void move_robot()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        // Randomly choose between different movement styles
        int action = random_action_(rng_);
        
        switch (action)
        {
            case 0: // Move forward long
                msg.linear.x = 0.3;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving forward...");
                rclcpp::sleep_for(5s);
                break;

            case 1: // Turn randomly
                msg.linear.x = 0.0;
                msg.angular.z = (random_action_(rng_) % 2 == 0) ? 0.5 : -0.5; // Random left/right turn
                RCLCPP_INFO(this->get_logger(), "Turning randomly...");
                rclcpp::sleep_for(2s);
                break;

            case 2: // Zigzag motion
                for (int i = 0; i < 3; ++i)
                {
                    msg.linear.x = 0.2;
                    msg.angular.z = 0.3;
                    cmd_vel_pub_->publish(msg);
                    rclcpp::sleep_for(1s);

                    msg.angular.z = -0.3;
                    cmd_vel_pub_->publish(msg);
                    rclcpp::sleep_for(1s);
                }
                break;

            case 3: // Stop randomly
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Pausing...");
                rclcpp::sleep_for(3s);
                break;
        }

        // Publish movement command
        cmd_vel_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleBot3PathPlanner>());
    rclcpp::shutdown();
    return 0;
}

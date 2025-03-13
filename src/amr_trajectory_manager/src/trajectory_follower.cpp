#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <fstream>
#include <sstream>
#include "amr_trajectory_msgs/msg/trajectory_data.hpp"

class TrajectoryReader : public rclcpp::Node {
public:
    TrajectoryReader() : Node("trajectory_reader") {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("trajectory_file", "");
        this->declare_parameter<std::string>("trajectory_topic", "/trajectory_data");
        this->declare_parameter<std::string>("marker_topic", "/trajectory_markers");

        file_path_ = this->get_parameter("trajectory_file").as_string();
        trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();

        // Publishers
        trajectory_pub_ = this->create_publisher<amr_trajectory_msgs::msg::TrajectoryData>(trajectory_topic_, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

        // TF Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Read and publish trajectory
        read_and_publish();
    }

private:
    void read_and_publish() {
        if (file_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory file path not provided.");
            return;
        }

        std::ifstream file(file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", file_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Opening file: %s", file_path_.c_str());

        std::string line;
        std::getline(file, line); // Skip header

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        rclcpp::Rate loop_rate(50); // 50Hz publishing rate

        while (rclcpp::ok() && std::getline(file, line)) {
            std::stringstream ss(line);
            std::vector<std::string> values;
            std::string value;

            while (std::getline(ss, value, ',')) {
                values.push_back(value);
            }

            if (values.size() < 14) continue;

            auto msg = std::make_shared<amr_trajectory_msgs::msg::TrajectoryData>();

            double full_time = std::stod(values[0]);
            msg->timestamp.sec = static_cast<int>(full_time);
            msg->timestamp.nanosec = static_cast<int>((full_time - msg->timestamp.sec) * 1e9);

            msg->pose.position.x = std::stod(values[1]);
            msg->pose.position.y = std::stod(values[2]);
            msg->pose.position.z = std::stod(values[3]);
            msg->pose.orientation.x = std::stod(values[4]);
            msg->pose.orientation.y = std::stod(values[5]);
            msg->pose.orientation.z = std::stod(values[6]);
            msg->pose.orientation.w = std::stod(values[7]);

            msg->velocity.linear.x = std::stod(values[8]);
            msg->velocity.linear.y = std::stod(values[9]);
            msg->velocity.linear.z = std::stod(values[10]);
            msg->velocity.angular.x = std::stod(values[11]);
            msg->velocity.angular.y = std::stod(values[12]);
            msg->velocity.angular.z = std::stod(values[13]);

            trajectory_pub_->publish(*msg);

            // Create a marker for visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = msg->pose;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
            marker_pub_->publish(marker_array);

            loop_rate.sleep();
        }
        file.close();
    }

    std::string file_path_;
    std::string trajectory_topic_;
    std::string marker_topic_;

    rclcpp::Publisher<amr_trajectory_msgs::msg::TrajectoryData>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryReader>());
    rclcpp::shutdown();
    return 0;
}

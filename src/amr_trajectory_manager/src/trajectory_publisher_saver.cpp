#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <amr_trajectory_msgs/srv/save_trajectory.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <filesystem>

class TrajectoryLogger : public rclcpp::Node {
public:
    TrajectoryLogger() : Node("trajectory_logger"), is_logging_(false), max_duration_(0.0) {
        // Declare parameters with default values
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("marker_topic", "/trajectory_markers");
        this->declare_parameter<std::string>("trajectory_directory", "/home/user/trajectories");
        
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string marker_topic = this->get_parameter("marker_topic").as_string();
        trajectory_directory_ = this->get_parameter("trajectory_directory").as_string();

        // Subscribe to odometry topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&TrajectoryLogger::odom_callback, this, std::placeholders::_1)
        );

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic, 10
        );

        // Service to save trajectory
        save_service_ = this->create_service<amr_trajectory_msgs::srv::SaveTrajectory>(
            "save_trajectory", 
            std::bind(&TrajectoryLogger::save_trajectory, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!is_logging_) return;

        double time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (time - start_time_ > max_duration_) {
            is_logging_ = false;
            trajectory_data_.clear();
            file_.close();
            return;
        }

        TrajectoryData data = {time, msg->pose.pose.position, msg->pose.pose.orientation, 
                               msg->twist.twist.linear, msg->twist.twist.angular};
        trajectory_data_.push_back(data);
        
        while (!trajectory_data_.empty() && time - trajectory_data_.front().time > max_duration_) {
            trajectory_data_.erase(trajectory_data_.begin());
        }

        if (file_.is_open()) {
            file_.seekp(0);
            file_ << "time,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
            for (const auto& d : trajectory_data_) {
                file_ << std::fixed << std::setprecision(6)
                      << d.time << "," << d.position.x << "," << d.position.y << "," << d.position.z << ","
                      << d.orientation.x << "," << d.orientation.y << "," << d.orientation.z << "," << d.orientation.w << ","
                      << d.linear_velocity.x << "," << d.linear_velocity.y << "," << d.linear_velocity.z << ","
                      << d.angular_velocity.x << "," << d.angular_velocity.y << "," << d.angular_velocity.z << "\n";
            }
        }
        
        publish_marker_array();
    }

void save_trajectory(const std::shared_ptr<amr_trajectory_msgs::srv::SaveTrajectory::Request> request,
                     std::shared_ptr<amr_trajectory_msgs::srv::SaveTrajectory::Response> response) {
    is_logging_ = true;
    start_time_ = this->now().seconds();
    max_duration_ = request->duration;

    // Construct the full file path
    std::string full_path = trajectory_directory_ + "/" + request->filename;

    // Remove existing file if it exists
    std::remove(full_path.c_str());

    // Open file for writing
    file_.open(full_path, std::ios::out);
    
    if (!file_) {
        response->success = false;
        response->message = "Failed to open file: " + full_path;
        return;
    }

    // Write CSV header
    file_ << "time,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
    response->success = true;
    response->message = "Trajectory logging started at: " + full_path;
}

    void publish_marker_array() {
        if (!is_logging_) return;

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto &data : trajectory_data_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = data.position;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    struct TrajectoryData {
        double time;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;
        geometry_msgs::msg::Vector3 linear_velocity;
        geometry_msgs::msg::Vector3 angular_velocity;
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<amr_trajectory_msgs::srv::SaveTrajectory>::SharedPtr save_service_;

    std::vector<TrajectoryData> trajectory_data_;
    bool is_logging_;
    double start_time_;
    double max_duration_;
    std::ofstream file_;
    std::string trajectory_directory_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryLogger>());
    rclcpp::shutdown();
    return 0;
}

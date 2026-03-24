#ifndef TERMINAL_PCL_VISUALIZER_TERMINAL_PCL_NODE_HPP_
#define TERMINAL_PCL_VISUALIZER_TERMINAL_PCL_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "terminal_pcl_visualizer/types.hpp"

namespace terminal_pcl_visualizer {

class TerminalPCLNode : public rclcpp::Node {
public:
    explicit TerminalPCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    std::shared_ptr<CloudData> get_data();
    void send_command(double linear_x, double angular_z);
    bool is_teleop_enabled() const { return publish_cmd_vel_; }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    bool publish_cmd_vel_;
    std::mutex mtx_;
    std::shared_ptr<CloudData> data_;
};

} // namespace terminal_pcl_visualizer

#endif // TERMINAL_PCL_VISUALIZER_TERMINAL_PCL_NODE_HPP_

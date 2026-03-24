#include "terminal_pcl_visualizer/terminal_pcl_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>

namespace terminal_pcl_visualizer {

TerminalPCLNode::TerminalPCLNode(const rclcpp::NodeOptions & options)
: Node("terminal_pcl_visualizer", options) {
    this->declare_parameter("topic", "/points");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("max_points", 20000);
    this->declare_parameter("enable_teleop", false);

    std::string topic = this->get_parameter("topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_vel_topic").as_string();
    publish_cmd_vel_ = this->get_parameter("enable_teleop").as_bool();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            this->callback(msg);
        });
    
    if (publish_cmd_vel_) {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    }
    
    data_ = std::make_shared<CloudData>();
    data_->frame_id = "waiting...";
}

std::shared_ptr<CloudData> TerminalPCLNode::get_data() {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

void TerminalPCLNode::send_command(double linear_x, double angular_z) {
    if (!publish_cmd_vel_ || !cmd_pub_) return;
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_pub_->publish(msg);
}

void TerminalPCLNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto next = std::make_shared<CloudData>();
    next->frame_id = msg->header.frame_id;

    size_t max_p = static_cast<size_t>(this->get_parameter("max_points").as_int());
    size_t total_p = static_cast<size_t>(msg->width) * msg->height;
    
    try {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        size_t step = std::max<size_t>(1, total_p / max_p);
        double sx = 0, sy = 0, sz = 0;

        for (size_t i = 0; i < total_p; i += step) {
            if (!(it_x != it_x.end())) break;
            float x = *it_x, y = *it_y, z = *it_z;
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                next->points.push_back({x, y, z});
                sx += x; sy += y; sz += z;
                if (next->points.size() == 1) {
                    next->min_x = next->max_x = x;
                    next->min_y = next->max_y = y;
                    next->min_z = next->max_z = z;
                } else {
                    next->min_x = std::min(next->min_x, x); next->max_x = std::max(next->max_x, x);
                    next->min_y = std::min(next->min_y, y); next->max_y = std::max(next->max_y, y);
                    next->min_z = std::min(next->min_z, z); next->max_z = std::max(next->max_z, z);
                }
            }
            for (size_t s = 0; s < step && it_x != it_x.end(); ++s) { ++it_x; ++it_y; ++it_z; }
            if (next->points.size() >= max_p) break;
        }
        if (!next->points.empty()) {
            next->cx = sx / next->points.size();
            next->cy = sy / next->points.size();
            next->cz = sz / next->points.size();
        }
    } catch (...) {}

    {
        std::lock_guard<std::mutex> lock(mtx_);
        data_ = std::move(next);
    }
}

} // namespace terminal_pcl_visualizer

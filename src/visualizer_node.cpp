#include <csignal>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "terminal_pcl_visualizer/terminal_pcl_node.hpp"
#include "terminal_pcl_visualizer/visualizer.hpp"

using namespace terminal_pcl_visualizer;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TerminalPCLNode>();
    auto visualizer = std::make_shared<Visualizer>(node);

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    visualizer->run();

    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/dom/canvas.hpp"

using namespace ftxui;
using namespace std::chrono_literals;

struct Point { float x, y, z; };

class TerminalPCLNode : public rclcpp::Node {
public:
    TerminalPCLNode() : Node("terminal_pcl_visualizer") {
        this->declare_parameter("topic", "/points");
        this->declare_parameter("scale", 60.0);
        this->declare_parameter("max_points", 500);

        std::string topic = this->get_parameter("topic").as_string();
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->callback(msg);
            });
        
        data_ = std::make_shared<Data>();
        data_->frame_id = "waiting...";
    }

    struct Data {
        std::vector<Point> points;
        std::string frame_id;
        size_t count = 0;
        float center_x = 0, center_y = 0, center_z = 0;
        float min_x = 0, max_x = 0, min_z = 0, max_z = 0;
    };

    std::shared_ptr<Data> get_data() {
        std::lock_guard<std::mutex> lock(mtx_);
        return data_;
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto next = std::make_shared<Data>();
        next->frame_id = msg->header.frame_id;
        next->count = ++received_count_;

        size_t max_p = static_cast<size_t>(this->get_parameter("max_points").as_int());
        size_t total_p = static_cast<size_t>(msg->width) * msg->height;
        
        try {
            sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

            size_t step = std::max<size_t>(1, total_p / max_p);
            double sum_x = 0, sum_y = 0, sum_z = 0;

            for (size_t i = 0; i < total_p; i += step) {
                if (!(it_x != it_x.end())) break;
                float x = *it_x, y = *it_y, z = *it_z;
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                    next->points.push_back({x, y, z});
                    sum_x += x; sum_y += y; sum_z += z;
                    if (next->points.size() == 1) {
                        next->min_x = next->max_x = x;
                        next->min_z = next->max_z = z;
                    } else {
                        next->min_x = std::min(next->min_x, x); next->max_x = std::max(next->max_x, x);
                        next->min_z = std::min(next->min_z, z); next->max_z = std::max(next->max_z, z);
                    }
                }
                for (size_t s = 0; s < step && it_x != it_x.end(); ++s) { ++it_x; ++it_y; ++it_z; }
                if (next->points.size() >= max_p) break;
            }
            if (!next->points.empty()) {
                next->center_x = static_cast<float>(sum_x / next->points.size());
                next->center_y = static_cast<float>(sum_y / next->points.size());
                next->center_z = static_cast<float>(sum_z / next->points.size());
            }
        } catch (...) {}

        {
            std::lock_guard<std::mutex> lock(mtx_);
            data_ = std::move(next);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::mutex mtx_;
    std::shared_ptr<Data> data_;
    size_t received_count_ = 0;
};

std::atomic<bool> g_quit_flag{false};
void signal_handler(int) { g_quit_flag = true; }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = std::make_shared<TerminalPCLNode>();
    auto screen = ScreenInteractive::TerminalOutput();
    screen.TrackMouse(false);

    std::thread ros_thread([&]() { rclcpp::spin(node); });
    std::atomic<float> manual_scale{0.0f};

    auto renderer = Renderer([&]() {
        auto data = node->get_data();
        float base_scale = node->get_parameter("scale").as_double();
        float cur_manual = manual_scale.load();
        float scale = (cur_manual == 0.0f) ? base_scale : cur_manual;

        auto c = Canvas(200, 120);
        int cx = 100, cy = 60;

        // Draw white crosshair
        for(int i=-10; i<=10; ++i) c.DrawPoint(cx + i, cy, true, Color::White);
        for(int i=-6; i<=6; ++i) c.DrawPoint(cx, cy + i, true, Color::White);

        if (!data->points.empty()) {
            for (const auto& p : data->points) {
                int sx, sy;
                if (data->frame_id.find("camera") != std::string::npos) {
                    sx = cx + static_cast<int>((p.x - data->center_x) * scale);
                    sy = cy - static_cast<int>((p.z - data->center_z) * scale);
                } else {
                    sx = cx - static_cast<int>((p.y - data->center_y) * scale);
                    sy = cy - static_cast<int>((p.x - data->center_x) * scale);
                }

                if (sx >= 1 && sx < 199 && sy >= 1 && sy < 119) {
                    // Draw 2x2 block for high visibility
                    c.DrawPoint(sx, sy, true, Color::Yellow);
                    c.DrawPoint(sx+1, sy, true, Color::Yellow);
                    c.DrawPoint(sx, sy+1, true, Color::Yellow);
                    c.DrawPoint(sx+1, sy+1, true, Color::Yellow);
                }
            }
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << " Range X: " << (data->max_x - data->min_x) << "m, Z: " << (data->max_z - data->min_z) << "m";

        return window(text(" PCL Visualizer (Bird's Eye View) ") | hcenter | bold,
            vbox({
                hbox({
                    text(" Frame: " + data->frame_id) | color(Color::Cyan),
                    filler(),
                    text(ss.str()) | color(Color::GrayLight),
                    filler(),
                    text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green),
                    filler(),
                    text(" Scale: " + std::to_string((int)scale)) | color(Color::Yellow)
                }),
                separator(),
                canvas(std::move(c)) | hcenter | border,
                hbox({
                    text(" [+/-] Zoom | [r] Reset | [q] Quit ") | dim,
                    filler(),
                    text(!rclcpp::ok() ? " ROS SHUTDOWN " : (g_quit_flag ? " EXITING... " : "")) | bold | color(Color::Red)
                })
            })
        );
    });

    auto component = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Escape) { g_quit_flag = true; screen.Exit(); return true; }
        if (event == Event::Character('+') || event == Event::Character('=')) {
            float s = (manual_scale.load() == 0.0f) ? node->get_parameter("scale").as_double() : manual_scale.load();
            manual_scale = s * 1.2f; return true;
        }
        if (event == Event::Character('-') || event == Event::Character('_')) {
            float s = (manual_scale.load() == 0.0f) ? node->get_parameter("scale").as_double() : manual_scale.load();
            manual_scale = s / 1.2f; return true;
        }
        if (event == Event::Character('r')) { manual_scale = 0.0f; return true; }
        return false;
    });

    std::thread ui_thread([&]() {
        while (!g_quit_flag && rclcpp::ok()) {
            screen.PostEvent(Event::Custom);
            std::this_thread::sleep_for(50ms);
        }
        screen.Exit();
    });

    screen.Loop(component);

    g_quit_flag = true;
    if (ui_thread.joinable()) ui_thread.join();
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();

    return 0;
}

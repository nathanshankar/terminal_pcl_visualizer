#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>

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
        float center_y = 0, center_z = 0;
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
            double sum_y = 0, sum_z = 0;

            for (size_t i = 0; i < total_p; i += step) {
                if (!(it_x != it_x.end())) break;
                if (!std::isnan(*it_x) && !std::isnan(*it_y) && !std::isnan(*it_z)) {
                    next->points.push_back({*it_x, *it_y, *it_z});
                    sum_y += *it_y;
                    sum_z += *it_z;
                }
                for (size_t s = 0; s < step && it_x != it_x.end(); ++s) { ++it_x; ++it_y; ++it_z; }
                if (next->points.size() >= max_p) break;
            }
            if (!next->points.empty()) {
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

// Global atomic flag for quitting
std::atomic<bool> g_quit_flag{false};
void signal_handler(int) { g_quit_flag = true; }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = std::make_shared<TerminalPCLNode>();
    auto screen = ScreenInteractive::TerminalOutput(); // Most stable mode
    screen.TrackMouse(false); // No mouse track noise

    std::thread ros_thread([&]() { rclcpp::spin(node); });

    std::atomic<float> manual_scale{0.0f};

    auto renderer = Renderer([&]() {
        auto data = node->get_data();
        float base_scale = node->get_parameter("scale").as_double();
        float cur_manual = manual_scale.load();
        float scale = (cur_manual == 0.0f) ? base_scale : cur_manual;

        return window(text(" PCL Visualizer (X-Projection) ") | hcenter | bold,
            vbox({
                hbox({
                    text(" Frame: " + data->frame_id) | color(Color::Cyan),
                    filler(),
                    text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green),
                    filler(),
                    text(" Scale: " + std::to_string((int)scale)) | color(Color::Yellow)
                }),
                separator(),
                canvas([&](Canvas& c) {
                    int w = c.width(); int h = c.height();
                    int cx = w / 2; int cy = h / 2;
                    c.DrawPointLine(cx - 5, cy, cx + 5, cy, Color::GrayDark);
                    c.DrawPointLine(cx, cy - 3, cx, cy + 3, Color::GrayDark);

                    for (const auto& p : data->points) {
                        int sx = cx - static_cast<int>((p.y - data->center_y) * scale);
                        int sy = cy - static_cast<int>((p.z - data->center_z) * scale);
                        if (sx >= 0 && sx < w && sy >= 0 && sy < h) {
                            uint8_t d = std::min(255, static_cast<int>(std::abs(p.x) * 20));
                            c.DrawPoint(sx, sy, true, Color::RGB(255, 255 - d, 255 - d));
                        }
                    }
                }) | size(HEIGHT, EQUAL, 25) | border, // Fixed height for stability
                hbox({
                    text(" [+/-] Zoom | [r] Reset | [q] Quit ") | dim,
                    filler(),
                    text(g_quit_flag ? " EXITING... " : "") | bold | color(Color::Red)
                })
            })
        );
    });

    auto component = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Escape) { g_quit_flag = true; return true; }
        if (event == Event::Character('+') || event == Event::Character('=')) {
            float s = manual_scale.load();
            if (s == 0.0f) s = node->get_parameter("scale").as_double();
            manual_scale = s * 1.2f; return true;
        }
        if (event == Event::Character('-') || event == Event::Character('_')) {
            float s = manual_scale.load();
            if (s == 0.0f) s = node->get_parameter("scale").as_double();
            manual_scale = s / 1.2f; return true;
        }
        if (event == Event::Character('r')) { manual_scale = 0.0f; return true; }
        return false;
    });

    // Manual event loop to handle g_quit_flag
    std::thread ui_thread([&]() {
        while (!g_quit_flag && rclcpp::ok()) {
            screen.PostEvent(Event::Custom);
            std::this_thread::sleep_for(100ms);
        }
        screen.Exit();
    });

    screen.Loop(component);

    g_quit_flag = true;
    rclcpp::shutdown();
    if (ui_thread.joinable()) ui_thread.join();
    if (ros_thread.joinable()) ros_thread.join();

    return 0;
}

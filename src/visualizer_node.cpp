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
#include <cmath>

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
        this->declare_parameter("max_points", 2000);

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
        float cx = 0, cy = 0, cz = 0;
    };

    std::shared_ptr<Data> get_data() {
        std::lock_guard<std::mutex> lock(mtx_);
        return data_;
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto next = std::make_shared<Data>();
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::mutex mtx_;
    std::shared_ptr<Data> data_;
};

std::atomic<bool> g_quit_flag{false};
void signal_handler(int) { g_quit_flag = true; }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = std::make_shared<TerminalPCLNode>();
    auto screen = ScreenInteractive::TerminalOutput();

    // CALIBRATED DEFAULT VIEW (Based on user feedback)
    // Front view required 15 presses of A (-1.5 yaw) and 15 presses of E (+1.5 roll)
    const float DEF_YAW = -1.5f;
    const float DEF_PITCH = 0.0f;
    const float DEF_ROLL = 1.5f;
    const float DEF_DIST = 5.0f;

    std::atomic<float> yaw{DEF_YAW};
    std::atomic<float> pitch{DEF_PITCH};
    std::atomic<float> roll{DEF_ROLL};
    std::atomic<float> dist{DEF_DIST};
    std::atomic<float> zoom{250.0f};
    std::atomic<bool> cam_mode{true};

    std::thread ros_thread([&]() { rclcpp::spin(node); });

    auto renderer = Renderer([&]() {
        auto data = node->get_data();
        auto canvas_obj = Canvas(200, 120);
        int cx = 100, cy = 60;

        float cyaw = yaw.load();
        float cpitch = pitch.load();
        float croll = roll.load();
        float cdist = dist.load();
        float czoom = zoom.load();
        bool cm = cam_mode.load();

        if (!data->points.empty()) {
            for (const auto& p : data->points) {
                float dx = p.x - data->cx;
                float dy = p.y - data->cy;
                float dz = p.z - data->cz;

                float rx, ry, rz;
                if (cm) {
                    rx = dx; ry = dy; rz = dz;
                } else {
                    rx = -dy; ry = -dz; rz = dx;
                }

                // 1. Yaw
                float x1 = rx * cos(cyaw) + rz * sin(cyaw);
                float z1 = -rx * sin(cyaw) + rz * cos(cyaw);

                // 2. Pitch
                float y2 = ry * cos(cpitch) - z1 * sin(cpitch);
                float z2 = ry * sin(cpitch) + z1 * cos(cpitch);

                // 3. Roll
                float x3 = x1 * cos(croll) - y2 * sin(croll);
                float y3 = x1 * sin(croll) + y2 * cos(croll);

                float final_z = z2 + cdist;
                if (final_z > 0.1f) {
                    int sx = cx + static_cast<int>(czoom * x3 / final_z);
                    int sy = cy + static_cast<int>(czoom * y3 / final_z);
                    if (sx >= 0 && sx < 200 && sy >= 0 && sy < 120) {
                        canvas_obj.DrawPoint(sx, sy, true, Color::Yellow);
                    }
                }
            }
        }

        for(int i=-4; i<=4; ++i) {
            canvas_obj.DrawPoint(cx + i, cy, true, Color::White);
            canvas_obj.DrawPoint(cx, cy + i, true, Color::White);
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << "Y:" << cyaw << " P:" << cpitch << " R:" << croll;

        return window(text(" 3D PointCloud Visualizer ") | hcenter | bold,
            vbox({
                hbox({
                    text(" Frame: " + data->frame_id) | color(Color::Cyan),
                    filler(),
                    text(ss.str()) | color(Color::GrayLight),
                    filler(),
                    text(cm ? "[CAMERA]" : "[ROBOT]") | bold | color(Color::Magenta),
                    filler(),
                    text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green)
                }),
                separator(),
                canvas(std::move(canvas_obj)) | hcenter | border,
                hbox({
                    text(" [WASD] Orbit | [QE] Roll | [1/2/3] Views | [Q] Quit ") | dim,
                    filler(),
                    text(g_quit_flag ? " EXITING... " : "") | bold | color(Color::Red)
                })
            })
        );
    });

    auto component = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Escape) { g_quit_flag = true; screen.Exit(); return true; }
        if (event == Event::Character('w')) { pitch = pitch.load() - 0.1f; return true; }
        if (event == Event::Character('s')) { pitch = pitch.load() + 0.1f; return true; }
        if (event == Event::Character('a')) { yaw = yaw.load() - 0.1f; return true; }
        if (event == Event::Character('d')) { yaw = yaw.load() + 0.1f; return true; }
        if (event == Event::Character('e')) { roll = roll.load() + 0.1f; return true; }
        if (event == Event::Character('q')) { roll = roll.load() - 0.1f; return true; }
        if (event == Event::Character('c')) { cam_mode = !cam_mode.load(); return true; }
        
        // Calibrated Presets based on your offsets
        if (event == Event::Character('1')) { yaw = DEF_YAW; pitch = 0.0f; roll = DEF_ROLL; dist = DEF_DIST; return true; } // Front
        if (event == Event::Character('2')) { yaw = DEF_YAW; pitch = 1.5f; roll = DEF_ROLL; dist = 8.0f; return true; }     // Top
        if (event == Event::Character('3')) { yaw = DEF_YAW + 1.5f; pitch = 0.0f; roll = DEF_ROLL; dist = 8.0f; return true; } // Side
        
        if (event == Event::Character('r')) { yaw = DEF_YAW; pitch = DEF_PITCH; roll = DEF_ROLL; dist = DEF_DIST; return true; }
        if (event == Event::Character('+') || event == Event::Character('=')) { dist = dist.load() - 0.5f; return true; }
        if (event == Event::Character('-') || event == Event::Character('_')) { dist = dist.load() + 0.5f; return true; }
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

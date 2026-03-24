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
        this->declare_parameter("max_points", 20000); 

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
        float min_x = 0, max_x = 0;
        float min_y = 0, max_y = 0;
        float min_z = 0, max_z = 0;
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

    const float DEF_YAW = -1.5f;
    const float DEF_PITCH = 0.0f;
    const float DEF_ROLL = 1.5f;
    const float DEF_DIST = 5.0f;

    float cur_yaw = DEF_YAW;
    float cur_pitch = DEF_PITCH;
    float cur_roll = DEF_ROLL;
    float cur_dist = DEF_DIST;

    std::atomic<float> tar_yaw{DEF_YAW};
    std::atomic<float> tar_pitch{DEF_PITCH};
    std::atomic<float> tar_roll{DEF_ROLL};
    std::atomic<float> tar_dist{DEF_DIST};
    
    std::atomic<float> zoom{350.0f};
    std::atomic<bool> cam_mode{true};

    std::thread ros_thread([&]() { rclcpp::spin(node); });

    auto renderer = Renderer([&]() {
        auto data = node->get_data();
        const int sw = 220;
        const int sh = 110;
        auto c = Canvas(sw, sh);
        int cx = sw / 2;
        int cy = sh / 2;

        cur_yaw += (tar_yaw.load() - cur_yaw) * 0.2f;
        cur_pitch += (tar_pitch.load() - cur_pitch) * 0.2f;
        cur_roll += (tar_roll.load() - cur_roll) * 0.2f;
        cur_dist += (tar_dist.load() - cur_dist) * 0.2f;

        std::vector<float> z_buffer(sw * sh, 1000.0f);
        float x_range = std::max(0.1f, data->max_x - data->min_x);
        float czoom = zoom.load();
        bool cm = cam_mode.load();

        auto project = [&](float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) {
            float rx, ry, rz;
            if (cm) { rx = dx; ry = dy; rz = dz; }
            else { rx = -dy; ry = -dz; rz = dx; }

            float x1 = rx * cos(cur_yaw) + rz * sin(cur_yaw);
            float z1 = -rx * sin(cur_yaw) + rz * cos(cur_yaw);
            float y2 = ry * cos(cur_pitch) - z1 * sin(cur_pitch);
            float z2 = ry * sin(cur_pitch) + z1 * cos(cur_pitch);
            float x3 = x1 * cos(cur_roll) - y2 * sin(cur_roll);
            float y3 = x1 * sin(cur_roll) + y2 * cos(cur_roll);

            out_z = z2 + cur_dist;
            if (out_z > 0.1f) {
                out_sx = cx + static_cast<int>(czoom * x3 / out_z);
                out_sy = cy + static_cast<int>(czoom * y3 / out_z);
                return true;
            }
            return false;
        };

        auto draw_thick_point = [&](int sx, int sy, Color color) {
            if (sx >= 0 && sx < sw - 1 && sy >= 0 && sy < sh - 1) {
                c.DrawPoint(sx, sy, true, color);
                c.DrawPoint(sx+1, sy, true, color);
                c.DrawPoint(sx, sy+1, true, color);
                c.DrawPoint(sx+1, sy+1, true, color);
            }
        };

        // 1. Draw Grid on XY Plane, centered on data, anchored at actual cloud ground
        // Floor height: Use the minimum z value of the point cloud (data->min_z)
        float floor_val = data->min_z - data->cz;

        for (float g = -4.0f; g <= 4.0f; g += 1.0f) {
            for (float t = -4.0f; t <= 4.0f; t += 0.1f) {
            int sx, sy; float sz;
            // Grid follows the cloud center, floor at min_z
            if (project(t, g, floor_val, sx, sy, sz)) draw_thick_point(sx, sy, Color::GrayDark);
            if (project(g, t, floor_val, sx, sy, sz)) draw_thick_point(sx, sy, Color::GrayDark);
            }
        }

        // 2. Draw PointCloud
        if (!data->points.empty()) {
            for (const auto& p : data->points) {
                int sx, sy; float sz;
                if (project(p.x - data->cx, p.y - data->cy, p.z - data->cz, sx, sy, sz)) {
                    if (sx >= 1 && sx < sw - 1 && sy >= 1 && sy < sh - 1) {
                        if (sz < z_buffer[sy * sw + sx]) {
                            z_buffer[sy * sw + sx] = sz;
                            float v = std::clamp((p.x - data->min_x) / x_range, 0.0f, 1.0f);
                            uint8_t r_col = 0, g_col = 0, b_col = 0;
                            if (v < 0.25f) { r_col = 255; g_col = static_cast<uint8_t>(v * 1020); }
                            else if (v < 0.5f) { r_col = static_cast<uint8_t>((0.5f - v) * 1020); g_col = 255; }
                            else if (v < 0.75f) { g_col = 255; b_col = static_cast<uint8_t>((v - 0.5f) * 1020); }
                            else { g_col = static_cast<uint8_t>((1.0f - v) * 1020); b_col = 255; }
                            draw_thick_point(sx, sy, Color::RGB(r_col, g_col, b_col));
                        }
                    }
                }
            }
        }

        // Crosshair at the center of the point cloud, projected onto the floor (min_z)
        if (!data->points.empty()) {
            int sx, sy;
            float sz;
            // Project the center (cx, cy, min_z - cz) to screen coordinates
            if (project(data->min_x-data->cx, 0.0f, data->min_z - data->cz, sx, sy, sz)) {
            for (int i = -4; i <= 4; ++i) {
                c.DrawPoint(sx + i, sy, true, Color::White);
                c.DrawPoint(sx, sy + i, true, Color::White);
            }
            }
        }

        return window(text(" 3D High-Density Visualizer ") | hcenter | bold,
            vbox({
                hbox({
                    text(" Frame: " + data->frame_id) | color(Color::Cyan),
                    filler(),
                    text(cm ? "[CAMERA]" : "[ROBOT]") | bold | color(Color::Magenta),
                    filler(),
                    text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green)
                }),
                separator(),
                canvas(std::move(c)) | hcenter | size(HEIGHT, EQUAL, 30) | border,
                hbox({
                    text(" [WASD] Orbit | [OP] Roll | [1/2/3] Presets | [Q] Quit ") | dim,
                    filler(),
                    text(g_quit_flag ? " EXITING... " : "") | bold | color(Color::Red)
                })
            })
        );
    });

    auto component = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Escape) { g_quit_flag = true; screen.Exit(); return true; }
        if (event == Event::Character('a')) { tar_pitch = tar_pitch.load() - 0.2f; return true; }
        if (event == Event::Character('d')) { tar_pitch = tar_pitch.load() + 0.2f; return true; }
        if (event == Event::Character('w')) { tar_yaw = tar_yaw.load() - 0.2f; return true; }
        if (event == Event::Character('s')) { tar_yaw = tar_yaw.load() + 0.2f; return true; }
        if (event == Event::Character('p')) { tar_roll = tar_roll.load() + 0.2f; return true; }
        if (event == Event::Character('o')) { tar_roll = tar_roll.load() - 0.2f; return true; }
        if (event == Event::Character('c')) { cam_mode = !cam_mode.load(); return true; }
        
        if (event == Event::Character('1')) { tar_yaw = DEF_YAW; tar_pitch = 0.0f; tar_roll = DEF_ROLL; tar_dist = 5.0f; return true; }
        if (event == Event::Character('2')) { tar_yaw = DEF_YAW; tar_pitch = 1.5f; tar_roll = DEF_ROLL; tar_dist = 8.0f; return true; }
        if (event == Event::Character('3')) { tar_yaw = DEF_YAW - 1.5f; tar_pitch = 0.0f; tar_roll = DEF_ROLL; tar_dist = 8.0f; return true; }
        
        if (event == Event::Character('r')) { tar_yaw = DEF_YAW; tar_pitch = DEF_PITCH; tar_roll = DEF_ROLL; tar_dist = DEF_DIST; return true; }
        if (event == Event::Character('+') || event == Event::Character('=')) { tar_dist = tar_dist.load() - 0.5f; return true; }
        if (event == Event::Character('-') || event == Event::Character('_')) { tar_dist = tar_dist.load() + 0.5f; return true; }
        return false;
    });

    std::thread ui_thread([&]() {
        while (!g_quit_flag && rclcpp::ok()) {
            screen.PostEvent(Event::Custom);
            std::this_thread::sleep_for(30ms);
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

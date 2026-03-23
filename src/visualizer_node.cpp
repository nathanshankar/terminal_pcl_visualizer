#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
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
        this->declare_parameter("max_points", 300);
        
        std::string topic = this->get_parameter("topic").as_string();
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->callback(msg);
            });
        
        data_ = std::make_shared<Data>();
        data_->frame_id = "WAITING FOR DATA...";
    }

    struct Data {
        std::vector<Point> points;
        std::string frame_id;
        size_t count = 0;
        std::atomic<bool> updated{false};
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
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        size_t step = std::max<size_t>(1, total_p / max_p);
        for (size_t i = 0; i < total_p; i += step) {
            if (!(it_x != it_x.end())) break;
            if (*it_x > 0.05f) next->points.push_back({*it_x, *it_y, *it_z});
            for (size_t s = 0; s < step && it_x != it_x.end(); ++s) { ++it_x; ++it_y; ++it_z; }
            if (next->points.size() >= max_p) break;
        }

        next->updated = true;
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

// GLOBAL TO ENSURE CLEAN EXIT
std::function<void()> g_cleanup;
void handle_sigint(int) { if (g_cleanup) g_cleanup(); std::exit(0); }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    auto node = std::make_shared<TerminalPCLNode>();
    auto screen = ScreenInteractive::Fullscreen();
    screen.TrackMouse(false); // NO NOISE ON SCROLL
    g_cleanup = [&]() { screen.Exit(); };

    std::thread ros_thread([&]() { rclcpp::spin(node); });

    std::atomic<bool> quit{false};
    std::thread ui_thread([&]() {
        while (!quit && rclcpp::ok()) {
            screen.PostEvent(Event::Custom);
            std::this_thread::sleep_for(200ms); // 5 FPS is enough
        }
    });

    auto renderer = Renderer([&]() {
        auto data = node->get_data();
        double scale = node->get_parameter("scale").as_double();
        
        Elements header;
        if (data->count == 0) {
            header.push_back(text(" [ ERROR: NO DATA ON " + node->get_parameter("topic").as_string() + " ] ") | bold | color(Color::Red));
        } else {
            header.push_back(text(" Frame: " + data->frame_id) | color(Color::Cyan) | bold);
            header.push_back(filler());
            header.push_back(text(" Points: " + std::to_string(data->points.size())) | color(Color::Green));
        }

        return window(text(" PCL Terminal Visualizer ") | center | bold,
            vbox({
                hbox(std::move(header)),
                separator(),
                canvas([&](Canvas& c) {
                    int w = c.width(); int h = c.height();
                    int cx = w / 2; int cy = h / 2;
                    c.DrawPointLine(0, cy, w, cy, Color::GrayDark);
                    c.DrawPointLine(cx, 0, cx, h, Color::GrayDark);
                    for (const auto& p : data->points) {
                        int sx = cx - static_cast<int>(p.y * scale);
                        int sy = cy - static_cast<int>(p.z * scale);
                        if (sx >= 0 && sx < w && sy >= 0 && sy < h) {
                            uint8_t d = std::min(255, static_cast<int>(p.x * 12));
                            c.DrawPoint(sx, sy, true, Color::RGB(255 - d, 255, 255));
                        }
                    }
                }) | flex,
                separator(),
                text(" Controls: 'q' or 'Ctrl-C' to exit. Scroll is disabled. ") | dim
            })
        );
    });

    auto component = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Escape) {
            quit = true;
            screen.Exit();
            return true;
        }
        return false;
    });

    screen.Loop(component);

    quit = true;
    rclcpp::shutdown();
    if (ui_thread.joinable()) ui_thread.join();
    if (ros_thread.joinable()) ros_thread.join();
    return 0;
}

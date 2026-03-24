#ifndef TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_
#define TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "terminal_pcl_visualizer/terminal_pcl_node.hpp"

namespace terminal_pcl_visualizer {

class Visualizer {
public:
    explicit Visualizer(std::shared_ptr<TerminalPCLNode> node);
    void run();
    void stop();

private:
    std::shared_ptr<TerminalPCLNode> node_;
    ftxui::ScreenInteractive screen_;
    std::atomic<bool> quit_flag_{false};

    const float DEF_YAW = -1.5f;
    const float DEF_PITCH = 0.0f;
    const float DEF_ROLL = 1.5f;
    const float DEF_DIST = 5.0f;

    float cur_yaw_ = DEF_YAW;
    float cur_pitch_ = DEF_PITCH;
    float cur_roll_ = DEF_ROLL;
    float cur_dist_ = DEF_DIST;

    std::atomic<float> tar_yaw_{DEF_YAW};
    std::atomic<float> tar_pitch_{DEF_PITCH};
    std::atomic<float> tar_roll_{DEF_ROLL};
    std::atomic<float> tar_dist_{DEF_DIST};
    
    std::atomic<float> cam_x_{0.0f};
    std::atomic<float> cam_y_{0.0f};
    std::atomic<float> cam_z_{0.0f};
    
    std::atomic<float> zoom_{350.0f};
    std::atomic<bool> cam_mode_{true};

    float lin_vel_ = 0.5f;
    float ang_vel_ = 1.0f;

    ftxui::Element render_frame();
    bool handle_event(ftxui::Event event);
};

} // namespace terminal_pcl_visualizer

#endif // TERMINAL_PCL_VISUALIZER_VISUALIZER_HPP_

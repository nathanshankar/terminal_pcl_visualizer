#include "terminal_pcl_visualizer/visualizer.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

#include "ftxui/screen/terminal.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/dom/canvas.hpp"

using namespace ftxui;
using namespace std::chrono_literals;

namespace terminal_pcl_visualizer {

Visualizer::Visualizer(std::shared_ptr<TerminalPCLNode> node)
    : node_(node), screen_(ScreenInteractive::TerminalOutput()) {}

void Visualizer::run() {
    auto renderer = Renderer([this]() { return render_frame(); });
    auto component = CatchEvent(renderer, [this](Event event) { return handle_event(event); });

    std::thread ui_thread([&]() {
        while (!quit_flag_ && rclcpp::ok()) {
            screen_.PostEvent(Event::Custom);
            std::this_thread::sleep_for(30ms);
        }
        screen_.Exit();
    });

    screen_.Loop(component);
    quit_flag_ = true;
    if (ui_thread.joinable()) ui_thread.join();
}

void Visualizer::stop() {
    quit_flag_ = true;
    screen_.Exit();
}

Element Visualizer::render_frame() {
    auto data = node_->get_data();
    auto terminal = Terminal::Size();
    
    // FTXUI Canvas uses Braille dots (2x4 per char)
    // We reserve some height for header (2-3) and footer (1-2) and borders
    const int target_height = std::max(10, terminal.dimy - 6);
    const int target_width = std::max(10, terminal.dimx - 2);
    
    const int sw = target_width * 2;
    const int sh = target_height * 4;
    
    auto c = Canvas(sw, sh);
    int cx = sw / 2;
    int cy = sh / 2;

    cur_yaw_ += (tar_yaw_.load() - cur_yaw_) * 0.2f;
    cur_pitch_ += (tar_pitch_.load() - cur_pitch_) * 0.2f;
    cur_roll_ += (tar_roll_.load() - cur_roll_) * 0.2f;
    cur_dist_ += (tar_dist_.load() - cur_dist_) * 0.2f;

    std::vector<float> z_buffer(sw * sh, 1000.0f);
    float x_range = std::max(0.1f, data->max_x - data->min_x);
    float czoom = zoom_.load();
    bool cm = cam_mode_.load();

    const float s_yaw = std::sin(cur_yaw_);
    const float c_yaw = std::cos(cur_yaw_);
    const float s_pitch = std::sin(cur_pitch_);
    const float c_pitch = std::cos(cur_pitch_);
    const float s_roll = std::sin(cur_roll_);
    const float c_roll = std::cos(cur_roll_);

    float cx_off = cam_x_.load();
    float cy_off = cam_y_.load();
    float cz_off = cam_z_.load();

    auto project = [&](float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) {
        float rx, ry, rz;
        // Apply camera translation offset
        float tx = dx - cx_off;
        float ty = dy - cy_off;
        float tz = dz - cz_off;

        if (cm) { rx = tx; ry = ty; rz = tz; }
        else { rx = -ty; ry = -tz; rz = tx; }

        float x1 = rx * c_yaw + rz * s_yaw;
        float z1 = -rx * s_yaw + rz * c_yaw;
        float y2 = ry * c_pitch - z1 * s_pitch;
        float z2 = ry * s_pitch + z1 * c_pitch;
        float x3 = x1 * c_roll - y2 * s_roll;
        float y3 = x1 * s_roll + y2 * c_roll;

        out_z = z2 + cur_dist_;
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

    float floor_val = data->min_z - data->cz;
    for (float g = -4.0f; g <= 4.0f; g += 1.0f) {
        for (float t = -4.0f; t <= 4.0f; t += 0.1f) {
            int sx, sy; float sz;
            if (project(t, g, floor_val, sx, sy, sz)) draw_thick_point(sx, sy, Color::GrayDark);
            if (project(g, t, floor_val, sx, sy, sz)) draw_thick_point(sx, sy, Color::GrayDark);
        }
    }

    if (!data->points.empty()) {
        for (const auto& p : data->points) {
            int sx, sy; float sz;
            if (project(p.x - data->cx, p.y - data->cy, p.z - data->cz, sx, sy, sz)) {
                if (sx >= 0 && sx < sw - 1 && sy >= 0 && sy < sh - 1) {
                    float v = std::clamp((p.x - data->min_x) / x_range, 0.0f, 1.0f);
                    uint8_t r_col = 0, g_col = 0, b_col = 0;
                    if (v < 0.25f) { r_col = 255; g_col = static_cast<uint8_t>(v * 1020); }
                    else if (v < 0.5f) { r_col = static_cast<uint8_t>((0.5f - v) * 1020); g_col = 255; }
                    else if (v < 0.75f) { g_col = 255; b_col = static_cast<uint8_t>((v - 0.5f) * 1020); }
                    else { g_col = static_cast<uint8_t>((1.0f - v) * 1020); b_col = 255; }
                    Color col = Color::RGB(r_col, g_col, b_col);

                    auto z_plot = [&](int x, int y) {
                        int idx = y * sw + x;
                        if (sz < z_buffer[idx]) {
                            z_buffer[idx] = sz;
                            c.DrawPoint(x, y, true, col);
                        }
                    };
                    z_plot(sx, sy);
                    z_plot(sx + 1, sy);
                    z_plot(sx, sy + 1);
                    z_plot(sx + 1, sy + 1);
                }
            }
        }
    }

    if (!data->points.empty()) {
        int sx, sy; float sz;
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
                text(node_->is_teleop_enabled() ? "[TELEOP: ON]" : "[TELEOP: OFF]") | bold | color(node_->is_teleop_enabled() ? Color::Green : Color::Red),
                filler(),
                text(" Pts: " + std::to_string(data->points.size())) | color(Color::Green)
            }),
            separator(),
            canvas(std::move(c)) | hcenter | flex | border,
            hbox({
                text(" [WASD] Orbit | [ARROWS/PgUp/PgDn] Pan | [U-O/J-L/M-.] Teleop | [1-3] Presets ") | dim,
                filler(),
                text(quit_flag_ ? " EXITING... " : "") | bold | color(Color::Red)
            })
        })
    );
}

bool Visualizer::handle_event(Event event) {
    if (event == Event::Character('q') || event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }

    // Teleop logic (similar to teleop_twist_keyboard)
    if (event == Event::Character('i')) { node_->send_command(lin_vel_, 0.0); return true; }
    if (event == Event::Character('u')) { node_->send_command(lin_vel_, ang_vel_); return true; }
    if (event == Event::Character('o')) { node_->send_command(lin_vel_, -ang_vel_); return true; }
    if (event == Event::Character('j')) { node_->send_command(0.0, ang_vel_); return true; }
    if (event == Event::Character('l')) { node_->send_command(0.0, -ang_vel_); return true; }
    if (event == Event::Character('k')) { node_->send_command(0.0, 0.0); return true; }
    if (event == Event::Character('m')) { node_->send_command(-lin_vel_, -ang_vel_); return true; }
    if (event == Event::Character(',')) { node_->send_command(-lin_vel_, 0.0); return true; }
    if (event == Event::Character('.')) { node_->send_command(-lin_vel_, ang_vel_); return true; }

    if (event == Event::Character('y')) { lin_vel_ *= 1.1; ang_vel_ *= 1.1; return true; }
    if (event == Event::Character('h')) { lin_vel_ *= 0.9; ang_vel_ *= 0.9; return true; }

    // Camera Panning/Translation
    if (event == Event::ArrowUp)    { cam_z_ = cam_z_.load() + 0.2f; return true; }
    if (event == Event::ArrowDown)  { cam_z_ = cam_z_.load() - 0.2f; return true; }
    if (event == Event::ArrowLeft)  { cam_y_ = cam_y_.load() + 0.2f; return true; }
    if (event == Event::ArrowRight) { cam_y_ = cam_y_.load() - 0.2f; return true; }
    if (event == Event::PageUp)     { cam_x_ = cam_x_.load() + 0.2f; return true; }
    if (event == Event::PageDown)   { cam_x_ = cam_x_.load() - 0.2f; return true; }

    if (event == Event::Character('a')) { tar_pitch_ = tar_pitch_.load() - 0.2f; return true; }
    if (event == Event::Character('d')) { tar_pitch_ = tar_pitch_.load() + 0.2f; return true; }
    if (event == Event::Character('w')) { tar_yaw_ = tar_yaw_.load() - 0.2f; return true; }
    if (event == Event::Character('s')) { tar_yaw_ = tar_yaw_.load() + 0.2f; return true; }
    if (event == Event::Character('p')) { tar_roll_ = tar_roll_.load() + 0.2f; return true; }
    if (event == Event::Character('o')) { tar_roll_ = tar_roll_.load() - 0.2f; return true; }
    if (event == Event::Character('c')) { cam_mode_ = !cam_mode_.load(); return true; }
    
    if (event == Event::Character('1')) { tar_yaw_ = DEF_YAW; tar_pitch_ = 0.0f; tar_roll_ = DEF_ROLL; tar_dist_ = 5.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    if (event == Event::Character('2')) { tar_yaw_ = DEF_YAW; tar_pitch_ = 1.5f; tar_roll_ = DEF_ROLL; tar_dist_ = 8.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    if (event == Event::Character('3')) { tar_yaw_ = DEF_YAW - 1.5f; tar_pitch_ = 0.0f; tar_roll_ = DEF_ROLL; tar_dist_ = 8.0f; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    
    if (event == Event::Character('r')) { tar_yaw_ = DEF_YAW; tar_pitch_ = DEF_PITCH; tar_roll_ = DEF_ROLL; tar_dist_ = DEF_DIST; cam_x_ = 0.0f; cam_y_ = 0.0f; cam_z_ = 0.0f; return true; }
    if (event == Event::Character('+') || event == Event::Character('=')) { tar_dist_ = tar_dist_.load() - 0.5f; return true; }
    if (event == Event::Character('-') || event == Event::Character('_')) { tar_dist_ = tar_dist_.load() + 0.5f; return true; }
    return false;
}

} // namespace terminal_pcl_visualizer

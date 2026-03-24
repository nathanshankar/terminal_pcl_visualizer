#ifndef TERMINAL_PCL_VISUALIZER_TYPES_HPP_
#define TERMINAL_PCL_VISUALIZER_TYPES_HPP_

#include <vector>
#include <string>

namespace terminal_pcl_visualizer {

struct Point {
    float x, y, z;
};

struct CloudData {
    std::vector<Point> points;
    std::string frame_id;
    float cx = 0, cy = 0, cz = 0;
    float min_x = 0, max_x = 0;
    float min_y = 0, max_y = 0;
    float min_z = 0, max_z = 0;
};

} // namespace terminal_pcl_visualizer

#endif // TERMINAL_PCL_VISUALIZER_TYPES_HPP_

#include <librealsense2/rs.hpp>
#include <vector>
#include <string>
#include <thread>

class D435
{
public:
    D435(const std::string number);
    bool update();
    std::string get_number();
    std::vector<rs2::points> get_points() { return points; };
    rs2::video_frame get_color() { return color; };

private:
    rs2::pipeline p;
    std::string serial_number;
    std::vector<rs2::points> points;
    rs2::video_frame color = rs2::video_frame(rs2::frame());

    const int CAPACITY = 5;
    std::thread *t;
    rs2::frame_queue queue = rs2::frame_queue(CAPACITY);

    rs2::spatial_filter spatialFilter;
    rs2::frame spatial_filter(rs2::frame, rs2_option, float);

    rs2::decimation_filter decimationFilter;
    rs2::frame decimation_filter(rs2::frame, rs2_option, float);
};
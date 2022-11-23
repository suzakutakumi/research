#include "d435.hpp"

// シリアルナンバーを受け取り、初期化する
D435::D435(const std::string number)
{
    rs2::config conf;
    conf.enable_device(number);
    serial_number=number;
    conf.enable_stream(RS2_STREAM_COLOR);
    conf.enable_stream(RS2_STREAM_INFRARED);
    conf.enable_stream(RS2_STREAM_DEPTH);
    p.start(conf);
}

void D435::update()
{
    std::vector<std::vector<float>> data;

    rs2::frameset frames = p.wait_for_frames();

    color =frames.get_color_frame();
    if (!color)
        color = frames.get_infrared_frame();
    
    rs2::pointcloud pc;
    pc.map_to(color);

    rs2::depth_frame depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    
}

std::string D435::get_number(){
    return serial_number;
}

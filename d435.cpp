#include "d435.hpp"
#include <iostream>

// シリアルナンバーを受け取り、初期化する
D435::D435(const std::string number)
{
    rs2::config conf;
    conf.enable_device(number);
    serial_number = number;
    conf.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    conf.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    conf.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    p.start(conf);
}

void D435::update()
{
    std::vector<std::vector<float>> data;

    for (int i = 0; i < 30; i++)
    {
        auto frames = p.wait_for_frames(); // Drop several frames for auto-exposure
    }

    rs2::frameset frames = p.wait_for_frames();

    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    color = aligned_frames.first(RS2_STREAM_COLOR);
    if (!color)
        color = frames.get_infrared_frame();

    rs2::pointcloud pc;
    pc.map_to(color);

    rs2::depth_frame depth = aligned_frames.get_depth_frame();
    points = pc.calculate(depth);
}

std::string D435::get_number()
{
    return serial_number;
}

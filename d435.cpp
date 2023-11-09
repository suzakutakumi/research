#include "d435.hpp"
#include <iostream>
#include <thread>

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

    t = new std::thread([&]()
                        {
    while (true)
    {
        auto frames = p.wait_for_frames();
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(frames);

        // static rs2::temporal_filter temp_filter(0.5, 20, 3);
        // aligned_frames=temp_filter.process(aligned_frames);

        queue.enqueue(aligned_frames);
    } });
    t->detach();
}

bool D435::update()
{
    points.clear();
    rs2::frameset frames;
    if (!queue.poll_for_frame(&frames))
    {
        return false;
    }

    frames.get_data();
    color = frames.first(RS2_STREAM_COLOR);
    if (!color)
        color = frames.get_infrared_frame();

    rs2::pointcloud pc;
    pc.map_to(color);

    rs2::depth_frame depth = frames.get_depth_frame();

    points.push_back(pc.calculate(depth));

    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 1)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 2)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 3)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 4)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 5)));

    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.25)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.3)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5)));
    // points.push_back(pc.calculate(spatial_filter(depth, RS2_OPTION_FILTER_SMOOTH_ALPHA, 1.0)));

    // points.push_back(pc.calculate(decimation_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 2)));
    // points.push_back(pc.calculate(decimation_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 3)));
    // points.push_back(pc.calculate(decimation_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 4)));
    // points.push_back(pc.calculate(decimation_filter(depth, RS2_OPTION_FILTER_MAGNITUDE, 8)));

    static rs2::temporal_filter temp_filter1(0.4, 20, 3);
    points.push_back(pc.calculate(temp_filter1.process(depth)));

    return true;
}

std::string D435::get_number()
{
    return serial_number;
}

rs2::frame D435::spatial_filter(rs2::frame frame, rs2_option option, float value)
{
    spatialFilter.set_option(option, value);
    return spatialFilter.process(frame);
}

rs2::frame D435::decimation_filter(rs2::frame frame, rs2_option option, float value)
{
    decimationFilter.set_option(option, value);
    return decimationFilter.process(frame);
}
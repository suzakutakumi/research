#include "d435.hpp"

// シリアルナンバーを受け取り、初期化する
D435::D435(const std::string number)
{
    rs2::config conf;
    conf.enable_device(number);
    serial_number=number;
    p.start(conf);
}

std::vector<std::vector<float>> D435::update()
{
    std::vector<std::vector<float>> data;

    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    int width = depth.get_width();
    int height = depth.get_height();

    for (int i = 0; i < height; i++)
    {
        std::vector<float> row={};
        for (int j = 0; j < width; j++)
        {
            row.push_back(depth.get_distance(j, i));
        }
        data.push_back(row);
    }
    return data;
}

std::string D435::get_number(){
    return serial_number;
}

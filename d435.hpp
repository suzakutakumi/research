#pragma once
#include<librealsense2/rs.hpp>
#include<vector>
#include<string>

class D435{
    public:
        D435(const std::string);
        std::vector<std::vector<float>> update();
        std::string get_number();
    private:
        rs2::pipeline p;
        std::string serial_number;
};
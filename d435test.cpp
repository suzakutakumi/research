#include <iostream>
#include "d435.hpp"

enum Direction
{
    Front,
    Right,
    Left,
    Back,
};

int main(int argc, char *argv[])
try
{
    constexpr int num_of_device = 4;
    const std::string serial_numbers[num_of_device] = {
        "102422073987",
        "102422070478",
        "102122072472",
        "102422071935"};
    D435 *cameras[num_of_device];
    
    for (int i = 0; i < num_of_device; i++)
    {
        cameras[i] = new D435(serial_numbers[i]);
    }

    for (auto &c : cameras)
    {
        auto data = c->update();
        auto vertices=data.get_vertices();
        for(int i=0;i<data.size();i++){
            std::cout<<vertices[i].x<<","<<vertices[i].y<<","<<vertices[i].z<<std::endl;
        }
        delete c;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
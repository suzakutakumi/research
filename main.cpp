#include <iostream>
#include "d435.hpp"
#include "PointCloud.hpp"

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
        "102422073987", "102422070478", "102122072472", "102422071935"};
    D435 *cameras[num_of_device];

    int num_of_using_device = num_of_device;
    if (argc >= 2)
    {
        num_of_using_device = std::atoi(argv[1]);
    }

    for (int i = 0; i < num_of_using_device; i++)
    {
        cameras[i] = new D435(serial_numbers[i]);
    }

    PointCloud pcs[4];
    for (int i = 0; i < num_of_using_device; i++)
    {
        cameras[i]->update();

        auto points = cameras[i]->get_points();
        auto color = cameras[i]->get_color();
        pcs[i] = PointCloud(points, color);
        delete cameras[i];
    }

    PointCloud merged;
    for (int i = 0; i < num_of_using_device; i++)
    {
        std::string name;
        switch (i)
        {
        case Direction::Front:
            name = "front";
            pcs[i].filter([](pcl::PointXYZRGB &p)
                          { p.z += 0.045; });
            break;
        case Direction::Back:
            name = "back";
            pcs[i].filter([](pcl::PointXYZRGB &p)
                          {
                p.x=-p.x;
                p.z=-p.z;
                p.z-=0.045; });
            break;
        case Direction::Right:
            name = "right";
            pcs[i].filter([](pcl::PointXYZRGB &p)
                          {
                float x=p.x;
                p.x=p.z;
                p.z=-x;
                p.x+=0.045; });
            break;
        case Direction::Left:
            name = "left";
            pcs[i].filter([](pcl::PointXYZRGB &p)
                          {
                float x=p.x;
                p.x=-p.z;
                p.z=x;
                p.x-=0.045; });
            break;
        }
        merged.extended(pcs[i]);

        pcs[i].save_to_pcd(name);
        std::cout << name + ".pcd is saved" << std::endl;
    }

    merged.save_to_pcd("merged");

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
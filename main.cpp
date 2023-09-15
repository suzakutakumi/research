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
    std::string name_end = "";
    if (argc >= 2)
    {
        // num_of_using_device = std::atoi(argv[1]);
        name_end = argv[1];
    }

    for (int i = 0; i < num_of_using_device; i++)
    {
        cameras[i] = new D435(serial_numbers[i]);
    }

    PointCloud pcs[4];
    while (true)
    {
        for (int i = 0; i < num_of_using_device; i++)
        {
            cameras[i]->update();

            auto points = cameras[i]->get_points();
            auto color = cameras[i]->get_color();

            pcs[i] = PointCloud(points, color);

            std::string name;
            switch (i)
            {
            case Direction::Front:
                name = "front";
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              { p.z += 0.080; });
                break;
            case Direction::Back:
                name = "back";
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                p.x=-p.x;
                p.z=-p.z;
                p.z-=0.080; });
                break;
            case Direction::Right:
                name = "right";
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                float x=p.x;
                p.x=p.z;
                p.z=-x;
                p.x+=0.080; });
                break;
            case Direction::Left:
                name = "left";
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                float x=p.x;
                p.x=-p.z;
                p.z=x;
                p.x-=0.080; });
                break;
            }

            pcs[i].save_to_pcd(name + name_end);
            std::cout << name + name_end + ".pcd is saved" << std::endl;
        }

        PointCloud merged;
        for (auto &p : pcs)
        {
            merged.extended(p);
        }

        merged.save_to_pcd("pcd/merged" + name_end);
        std::cout << "merged" + name_end + ".pcd is saved" << std::endl;
        break;
    }
    std::cout << "ok" << std::endl;
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
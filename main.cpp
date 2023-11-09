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
    constexpr int num_of_device = 1;
    const std::string pcd_path = "pcd/";
    const std::string serial_numbers[4] = {
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
        std::cout<<"cameras init"<<std::endl;
        cameras[i] = new D435(serial_numbers[i]);
    }

    for(int j=0;j<5;j++){
        std::cout<<j<<std::endl;
        for (int i = 0; i < num_of_using_device; i++)
        {
            while(!cameras[i]->update());
        }
    }

    PointCloud pcs[4];
    while (true)
    {
        
        for (int i = 0; i < num_of_using_device; i++)
        {
            std::cout<<"camera "<<i+1<<" get pointcloud"<<std::endl;
            while(!cameras[i]->update());

            auto points_list = cameras[i]->get_points();
            auto color = cameras[i]->get_color();
            std::cout << i << std::endl;

            std::string option_names[] = {
                "",
                // "magnitude1",
                // "magnitude2",
                // "magnitude3",
                // "magnitude4",
                // "magnitude5",
                // "alpha0.25",
                // "alpha0.3",
                // "alpha0.4",
                // "alpha0.5",
                // "alpha1.0",
                // "decimation2",
                // "decimation3",
                // "decimation4",
                // "decimation8",

                "Temporal",
            };
            std::cout<<"size:"<<points_list.size()<<std::endl;
            for (int j = 0; j < points_list.size(); j++)
            {
                auto points = points_list[j];
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

                pcs[i].save_to_pcd(pcd_path + name + name_end + option_names[j]);
                std::cout << name + name_end + option_names[j] + ".pcd is saved" << std::endl;
            }
        }
        break;

        // PointCloud merged;
        // for (auto &p : pcs)
        // {
        //     merged.extended(p);
        // }

        // // merged.save_to_pcd(pcd_path + "merged" + name_end);
        // std::cout << "merged" + name_end + ".pcd is saved" << std::endl;
        // break;
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
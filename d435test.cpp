#include<iostream>
#include<cstdio>
#include<librealsense2/rs.hpp>
#include <time.h>

int main(int argc, char * argv[]) try
{
    rs2::pipeline p;
    // Configure and start the pipeline
    rs2::context context;
    rs2::device_list dlist=context.query_devices();

    for(const rs2::device& device:dlist){
        // Check Device
        // "Platform Camera" is not RealSense Devices
        const std::string friendly_name = device.get_info( rs2_camera_info::RS2_CAMERA_INFO_NAME );
        std::cout<<friendly_name<<"\n";
    }

    p.start();

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);
        static int t=0;
        if(t!=0){
            printf("\033[10A\r");
        }
        t=1.0;
        for(int i=0;i<10;i++){
            for(int j=0;j<10;j++){
                double d=0.0;
                for(int m=0;m<height/10;m++){
                    for(int n=0;n<width/10;n++){
                        d+=depth.get_distance(j*width/10+n,i*height/10+m);
                    }
                }
                d/=height*width/100;
                printf("%6.3f ",d);
            }
            printf("\n");
        }
        // Print the distance
        //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
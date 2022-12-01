#include "PointCloud.hpp"
PointCloud::PointCloud(const rs2::points &points, const rs2::video_frame &color)
{
    cloud = pc_ptr(new pc);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        auto RGB_Color = PointCloud::RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>
    }
}

std::tuple<int, int, int> PointCloud::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width = texture.get_width();   // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index = (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t *>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

void PointCloud::save_to_pcd(const std::string &n)
{
    std::string name = n;

    pcl::PointCloud<pcl::PointXYZRGB> newCloud;

    if (name.size() <= 4 || name.substr(name.size() - 4) != ".pcd")
    {
        name = name + std::string(".pcd");
    }

    // pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
    // Cloud_Filter.setInputCloud(cloud);               // Input generated cloud to filter
    // Cloud_Filter.setFilterFieldName("z");            // Set field name to Z-coordinate
    // Cloud_Filter.setFilterLimits(0.0, 1.0);          // Set accepted interval values
    // Cloud_Filter.filter(newCloud);                   // Filtered Cloud Outputted

    pcl::io::savePCDFileBinary(name, *cloud);
}
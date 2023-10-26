#include <convex_plane_visualizer/convex_plane_visualizer.h>

namespace convex_plane
{
ConvexPlaneVisualizer::ConvexPlaneVisualizer(const rclcpp::NodeOptions option)
: rclcpp::Node("convex_plane_visualizer", option)
{
    sub_convex_plane_ = create_subscription<convex_plane_msgs::msg::ConvexPlanesWithGridMap>
    (
        "convex_plane_visualizer/input/planes", 1, std::bind(&ConvexPlaneVisualizer::callbackConvexPlane, this, std::placeholders::_1)
    );

    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "convex_plane_visualizer/output/marker", 1
    );
    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>
    (
        "convex_plane_visualizer/output/grid_map", 1
    );
}

ConvexPlaneVisualizer::~ConvexPlaneVisualizer() {}

void ConvexPlaneVisualizer::callbackConvexPlane(const convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr msg)
{
    std::vector<iris_2d::Region> regions;
    std::vector<Vector> normals;
    std::vector<int> labels;
    grid_map::GridMap map;

    convex_plane::ConvexPlaneConverter::fromMessage(*msg, map, regions, normals, labels);

    // for (size_t i=0; i<labels.size(); ++i)
    // {
    //     RCLCPP_INFO_STREAM(get_logger(), "label: " << labels[i]);
    //     RCLCPP_INFO_STREAM(get_logger(), "A: " << regions[i].getA());
    //     RCLCPP_INFO_STREAM(get_logger(), "b: " << regions[i].getB().transpose());
    //     RCLCPP_INFO_STREAM(get_logger(), "C: " << regions[i].getC());
    //     RCLCPP_INFO_STREAM(get_logger(), "d: " << regions[i].getD().transpose());
    //     RCLCPP_INFO_STREAM(get_logger(), "normal: " << normals[i].transpose());
    // }

    if (!map.exists("convex_planes"))
    {
        map.add("convex_planes", 0.0);
    }
    
    for (grid_map::GridMapIterator iter(map); !iter.isPastEnd(); ++iter)
    {
        grid_map::Position pos;
        map.getPosition(*iter, pos);
        for (size_t i=0; i<labels.size(); ++i)
        {
            if (((regions[i].getA()*pos - regions[i].getB()).array() > 0.0).any()) continue;

            map.at("convex_planes", *iter) = 255;
        }
    }

    // create image
    cv::Mat image, binary;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "convex_planes", CV_8UC1, binary);
    // grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "convex_planes", CV_8UC1, image);
    // int min_label = *std::min_element(labels.begin(), labels.end());
    // RCLCPP_INFO(get_logger(), "min label: %d", min_label);
    // cv::threshold(image, binary, double(min_label), 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // get contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    // RCLCPP_INFO(get_logger(), "Contour size: %d", contours.size());

    // create markers to show the contours
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(contours.size());
    for (size_t i=0; i<marker_array.markers.size(); ++i)
    {
        marker_array.markers[i].header = msg->map.header;
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_array.markers[i].action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers[i].scale.x = 0.05;
        marker_array.markers[i].scale.y = 0.01;
        marker_array.markers[i].scale.z = 0.01;
        marker_array.markers[i].color.r = 0.0;
        marker_array.markers[i].color.g = 1.0;
        marker_array.markers[i].color.b = 0.0;
        marker_array.markers[i].color.a = 1.0;
        marker_array.markers[i].lifetime = rclcpp::Duration(0.0);
    }
    grid_map::Index index;
    grid_map::Position3 pos;
    geometry_msgs::msg::Point geo_pos;
    for (size_t j=0; j<contours.size(); ++j)
    {
        const std::vector<cv::Point>& contour = contours[j];
        visualization_msgs::msg::Marker& marker = marker_array.markers[j];
        for (size_t i=0; i<contour.size(); ++i)
        {
            const cv::Point& point = contour[i];
            index(0) = point.y;
            index(1) = point.x;
            if (!map.getPosition3("elevation_smooth", index, pos))
            {
                RCLCPP_ERROR(get_logger(), "Failed to get position of contour");
            }
            
            geo_pos.x = pos(0);
            geo_pos.y = pos(1);
            geo_pos.z = pos(2);
            // RCLCPP_INFO_STREAM(get_logger(), "set point: " << pos.transpose());
            marker.points.push_back(geo_pos);
        }   
        const cv::Point& point = contour[0];
        index(0) = point.y;
        index(1) = point.x;
        map.getPosition3("elevation_smooth", index, pos);
        
        geo_pos.x = pos(0);
        geo_pos.y = pos(1);
        geo_pos.z = pos(2);

        marker.points.push_back(geo_pos);
    }

    grid_map_msgs::msg::GridMap::UniquePtr map_msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_marker_->publish(marker_array);
    pub_grid_map_->publish(std::move(map_msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(convex_plane::ConvexPlaneVisualizer)
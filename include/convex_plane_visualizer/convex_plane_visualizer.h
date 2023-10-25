#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <convex_plane_msgs/msg/convex_planes_with_grid_map.hpp>
#include <convex_plane_converter/convex_plane_converter.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <iris.h>

namespace convex_plane
{
class ConvexPlaneVisualizer: public rclcpp::Node
{
public: 
    ConvexPlaneVisualizer(const rclcpp::NodeOptions option=rclcpp::NodeOptions().use_intra_process_comms(true));
    ~ConvexPlaneVisualizer();

private:
    rclcpp::Subscription<convex_plane_msgs::msg::ConvexPlanesWithGridMap>::SharedPtr sub_convex_plane_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;

    void callbackConvexPlane(const convex_plane_msgs::msg::ConvexPlanesWithGridMap::UniquePtr msg);

    // void createMarker(visualization_msgs::msg::MarkerArray& msg, const iris::IRISRegion& region, const grid_map::GridMap& map);

};

}
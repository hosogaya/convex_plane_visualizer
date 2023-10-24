#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
class MarkerNode : public rclcpp::Node
{
public:
    MarkerNode(): rclcpp::Node("marker_node")
    {
        pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "marker", 1
        );
        timer_ = create_wall_timer(500ms, std::bind(&MarkerNode::callbackTimer, this));
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callbackTimer()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0.0);
        marker.points.resize(10);

        for (size_t i=0; i<marker.points.size(); ++i)
        {
            marker.points[i].x = i;
            marker.points[i].y = i+2;
            marker.points[i].z = i+2;
        }
        pub_->publish(marker);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerNode>());
    rclcpp::shutdown();
    
    return 0;
}
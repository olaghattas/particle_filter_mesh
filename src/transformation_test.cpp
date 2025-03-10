//
// Created by olagh48652 on 3/4/25.
//

#include <rclcpp/rclcpp.hpp>
//#include "geometry_msgs/msg/point_stamped.hpp"
//
//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/buffer.h"
//
//#include "zed_interfaces/msg/objects_stamped.hpp"
//#include "visualization_msgs/msg/marker.hpp"
#include "particle_filter/transformation_test.hpp"

//class PointTransformer : public rclcpp::Node{
//public:
//    PointTransformer() : rclcpp::Node("point_transformer")
//    {
//        // Create a TF2 buffer and listener for transformation
//        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//
//        // Create a publisher for the marker visualization
//        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
//
//
//        subscription_ = create_subscription<zed_interfaces::msg::ObjectsStamped>(
//                "/zed_doorway/zed_node_doorway/body_trk/skeletons", 1,
//                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { objects_callback(msg); });
//
//
//    }
//
//private:
//    void objects_callback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
//    {
//        if (!msg->objects.empty()) {
//            // Iterate through each detected object
//            for (const auto &obj: msg->objects) {
//                // Extract head position
//                const std::array<float, 3>& head_position = obj.head_position;
//
//                if (head_position.size() >= 3) {
////                    double x = head_position[0];
////                    double y = head_position[1];
////                    double z = head_position[2];
//                    float x = head_position[0];
//                    float y = head_position[1];
//                    float z = head_position[2];
//
//                    RCLCPP_INFO(this->get_logger(), "Head Position (zed_left_camera_frame): (%f, %f, %f)", x, y, z);
//
//                    // Now, transform the point to "zed_doorway_cam" frame
//                    transform_point(x, y, z);
//                }
//            }
//        }
//    }
//
//    void transform_point(float x, float y, float z)
//    {
//        try
//        {
//            // Create PointStamped message for 3D head position in "zed_left_camera_frame"
//            geometry_msgs::msg::PointStamped point_stamped;
//            point_stamped.header.frame_id = "zed_doorway_left_camera_frame";  // Source frame
//            point_stamped.header.stamp = this->get_clock()->now();
//
//            point_stamped.point.x = x;  // Implicit cast from double to float
//            point_stamped.point.y = y;
//            point_stamped.point.z = z;
//
//            // Inside transform_point():
////            if (!tf_buffer_->canTransform("zed_doorway_cam",
////                                          point_stamped.header.frame_id,
////                                          point_stamped.header.stamp,
////                                          tf2::Duration(100ms))) {
////                RCLCPP_WARN(this->get_logger(), "Transform not available");
////                return;
////            }
//
//            // Transform the point to "zed_doorway_cam"
//            geometry_msgs::msg::PointStamped transformed_point;
//            transformed_point = tf_buffer_->transform(point_stamped, "zed_doorway_cam");
//
//            // Log the transformed point
//            RCLCPP_INFO(this->get_logger(), "Transformed Point (zed_doorway_cam): (%f, %f, %f)",
//                        transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
//
//            // Publish the transformed point as a marker to visualize in Rviz
//            publish_marker(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
//        }
//        catch (const tf2::TransformException &e)
//        {
//            RCLCPP_ERROR(this->get_logger(), "Error transforming point: %s", e.what());
//        }
//        catch (const std::exception &e)
//        {
//            RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
//        }
//    }
//
//    void publish_marker(double x, double y, double z)
//    {
//        // Create a marker message to visualize the point in Rviz
//        visualization_msgs::msg::Marker marker;
////        auto marker = std::make_shared<visualization_msgs::msg::Marker>();
//
//        marker.header.frame_id = "zed_doorway_cam";  // Frame of reference for the marker
//        marker.header.stamp = this->get_clock()->now();
//        marker.ns = "head_position_marker";
//        marker.id = 0;  // Unique ID for the marker
//        marker.type = visualization_msgs::msg::Marker::SPHERE;  // Use a sphere to represent the point
//        marker.action = visualization_msgs::msg::Marker::ADD;
//        marker.pose.position.x = x;
//        marker.pose.position.y = y;
//        marker.pose.position.z = z;
//        marker.scale.x = 0.1;  // Size of the sphere
//        marker.scale.y = 0.1;
//        marker.scale.z = 0.1;
//        marker.color.a = 1.0;  // Transparency
//        marker.color.r = 1.0;  // Red color
//        marker.color.g = 0.0;
//        marker.color.b = 0.0;
//
//        // Publish the marker
//        marker_pub_->publish(marker);
//    }
//
//    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr subscription_;
//    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
//    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformer>());
    rclcpp::shutdown();
    return 0;
}

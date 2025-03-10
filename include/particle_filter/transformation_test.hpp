//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "visualization_msgs/msg/marker.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include <array>
#include <yaml-cpp/yaml.h>

#include "zed_interfaces/msg/object.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "builtin_interfaces/msg/time.hpp"

class PointTransformer : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr subscription_;


    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;


public:

    PointTransformer() : rclcpp::Node("point_transformer") {

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_doorway/zed_node_doorway/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_doorway(msg); });

    }


    void PosePixCallback_doorway(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {

        if (!msg->objects.empty()) {
            for (const auto &obj: msg->objects) {
                // Extract head position
                const std::array<float, 3>& head_position = obj.head_position;

                if (head_position.size() >= 3) {
//                    double x = head_position[0];
//                    double y = head_position[1];
//                    double z = head_position[2];
                    float x = head_position[0];
                    float y = head_position[1];
                    float z = head_position[2];

                    RCLCPP_INFO(this->get_logger(), "Head Position (zed_left_camera_frame): (%f, %f, %f)", x, y, z);

                    // Now, transform the point to "zed_doorway_cam" frame
                    transform_point(x, y, z);
                }
            }
        }
    }

    void publish_marker(float x, float y, float z) {
        auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        // Create a marker message for a single point
//        visualization_msgs::msg::Marker marker_msg;
        marker_msg->header.frame_id = "zed_doorway_cam"; // Replace with your desired frame_id
        marker_msg->id = 0;
        marker_msg->header.stamp = this->get_clock()->now();
        marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg->action = visualization_msgs::msg::Marker::ADD;
        marker_msg->scale.x = 0.1; // Scale factors for the point (adjust as needed)
        marker_msg->scale.y = 0.1;
        marker_msg->scale.z = 0.1;
        marker_msg->color.r = 1.0;
        marker_msg->color.g = 0;
        marker_msg->color.b = 0;
        marker_msg->color.a = 1.0;

        // Set the position of the point
        marker_msg->pose.position.x = x;
        marker_msg->pose.position.y = y;
        marker_msg->pose.position.z = z;

//         Publish the marker
        marker_pub_->publish(*marker_msg);
    }


//    void transform_point(float x, float y, float z) {
//        try {
//            // get the geometry transform frames
//            geometry_msgs::msg::PointStamped point_stamped;
//
//            point_stamped.header.frame_id = "zed_doorway_left_camera_frame";  // Source frame
//            point_stamped.header.stamp = this->get_clock()->now();
//
//            point_stamped.point.x = x;  // Implicit cast from double to float
//            point_stamped.point.y = y;
//            point_stamped.point.z = z;
//
//
//            = tf_buffer_->lookupTransform(
//                    toFrame, fromFrame,
//                    tf2::TimePoint(), std::chrono::milliseconds(100000));
//
//            geometry_msgs::msg::Transform transform_ = t.transform;
//
//            // turn geometry transform to 4x4 matrix
//            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform = transform_geometry_to_matrix(transform_);
//            RCLCPP_INFO(this->get_logger(), "transform %s to %s", fromFrame.c_str(), toFrame.c_str());
//
//            return transform;
//
//        }
//        catch (const tf2::TransformException &ex) {
//            RCLCPP_INFO(
//                    this->get_logger(), "Could not transform %s to %s: %s",
//                    fromFrame.c_str(), toFrame.c_str(), ex.what());
////            return;
//        }
//    }



    void transform_point(float x, float y, float z)
    {
        try
        {
            // Create PointStamped message for 3D head position in "zed_left_camera_frame"
            geometry_msgs::msg::PointStamped point_stamped;
            point_stamped.header.frame_id = "zed_doorway_left_camera_frame";  // Source frame
            point_stamped.header.stamp = this->get_clock()->now();

            point_stamped.point.x = x;  // Implicit cast from double to float
            point_stamped.point.y = y;
            point_stamped.point.z = z;

            // Inside transform_point():
//            if (!tf_buffer_->canTransform("zed_doorway_cam",
//                                          point_stamped.header.frame_id,
//                                          point_stamped.header.stamp,
//                                          tf2::Duration(100ms))) {
//                RCLCPP_WARN(this->get_logger(), "Transform not available");
//                return;
//            }

            // Transform the point to "zed_doorway_cam"
            geometry_msgs::msg::PointStamped transformed_point;
            tf_buffer_->transform(point_stamped,transformed_point, "zed_doorway_cam", tf2::Duration(std::chrono::milliseconds(200)));

            // Log the transformed point
            RCLCPP_INFO(this->get_logger(), "Transformed Point (zed_doorway_cam): (%f, %f, %f)",
                        transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

            // Publish the transformed point as a marker to visualize in Rviz
            publish_marker(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error transforming point: %s", e.what());
        }

    }



};

#include <unordered_map>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

const std::vector<std::string> lndmarks = {"obstacles", "bedroom", "bathroom", "main_door"};

const std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> transform_map = {
        {"obstacles", [](){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = rclcpp::Clock().now();
            t.header.frame_id = "unity";
            t.child_frame_id = "obstacles";
            t.transform.translation.x = 1.0;  // Set to the actual x position
            t.transform.translation.y = 2.0;  // Set to the actual y position
            t.transform.translation.z = 0.0;  // Set to the actual z position
            t.transform.rotation.w = 1.0;  // No rotation
            return t;
        }()},
        {"bedroom", [](){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = rclcpp::Clock().now();
            t.header.frame_id = "unity";
            t.child_frame_id = "bedroom";
            t.transform.translation.x = 3.0;  // Set to the actual x position
            t.transform.translation.y = 4.0;  // Set to the actual y position
            t.transform.translation.z = 0.0;  // Set to the actual z position
            t.transform.rotation.w = 1.0;  // No rotation
            return t;
        }()},
        {"bathroom", [](){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = rclcpp::Clock().now();
            t.header.frame_id = "unity";
            t.child_frame_id = "bathroom";
            t.transform.translation.x = 5.0;  // Set to the actual x position
            t.transform.translation.y = 6.0;  // Set to the actual y position
            t.transform.translation.z = 0.0;  // Set to the actual z position
            t.transform.rotation.w = 1.0;  // No rotation
            return t;
        }()},
        {"main_door", [](){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = rclcpp::Clock().now();
            t.header.frame_id = "unity";
            t.child_frame_id = "main_door";
            t.transform.translation.x = 7.0;  // Set to the actual x position
            t.transform.translation.y = 8.0;  // Set to the actual y position
            t.transform.translation.z = 0.0;  // Set to the actual z position
            t.transform.rotation.w = 1.0;  // No rotation
            return t;
        }()},
};

int main() {
    // Example usage
    auto transform = transform_map.at("bedroom");
    // You can now use `transform`, e.g., broadcasting it
    // tf_broadcaster_->sendTransform(transform);

    return 0;
}

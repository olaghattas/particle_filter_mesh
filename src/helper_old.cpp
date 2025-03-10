//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
//#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/bool.hpp"

#include "shr_utils/geometry.hpp"

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


#include <map>
#include <opencv2/opencv.hpp>
#include <array>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class MeshNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped t;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;


public:

    std::unordered_map<std::string, std::vector<float>> mesh_vert_map_;
    bool flag_obs = false;
    MeshNode() : Node("particle_filter") {
        flag = create_subscription<std_msgs::msg::Bool>(
                "/flag", 1,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { FlagCallback(msg); });

        clicked_point = create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point", 1,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { ClickedPointCallback(msg); });

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter_mesh");
        auto mesh_file = (pkg_dir / "config" / "transition_2.obj").string();

        auto [mesh_verts, mesh_names] = shr_utils::load_meshes_squares(mesh_file);
        for (int i = 0; i < mesh_names.size(); i++) {
//            std::cout << "mesh name: " << mesh_names[i] << std::endl;
//            std::cout << "mesh vert: " << mesh_verts[i] << std::endl;
            auto name = mesh_names[i];
            auto verts = mesh_verts[i];
            mesh_vert_map_[name] = extractMinMax(verts);

        }
    }

    void FlagCallback(const std_msgs::msg::Bool::SharedPtr msg){

        if (msg->data) {
            // Do something if the message contains 'true'
            flag_obs = true;
        } else {
            // Do something if the message contains 'false'
            flag_obs = false;
        }
    }

    void publish_particles(std::vector<std::pair<float, float>> &particles) {       // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        // Populate the marker array with markers
        int count = 0;
        for (const auto &particle: particles) {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "unity";
            marker.header.stamp = this->get_clock()->now();
            marker.id = count++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = particle.first;
            marker.pose.position.y = particle.second;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            marker.scale.x = 0.05;  // Set the scale to make the arrow thinner
            marker.scale.y = 0.01;  // Set the scale to make the arrow thinner
            marker.scale.z = 0.01;  // Set the scale to make the arrow thinner
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Add the marker to the marker array
            markerArrayMsg->markers.push_back(marker);
        }
        // Publish the marker array
        publisher_->publish(*markerArrayMsg);

    }
    void ClickedPointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::cout << "msg.x " << msg->point.x << std::endl;
        std::cout << "msg.y " << msg->point.y << std::endl;


        t.header.stamp = rclcpp::Clock().now();
        t.header.frame_id = "map";
        /// should be whatever the code is expecting the name to be
        t.child_frame_id = "nathan";
        t.transform.translation.x = msg->point.x;
        t.transform.translation.y = msg->point.y;
        t.transform.translation.z = 0;
        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w = 1;
    }

    std::vector<float> extractMinMax(const Eigen::MatrixXd& matrix) {
        // Ensure the matrix has at least two rows
        if (matrix.rows() < 2) {
            throw std::runtime_error("Matrix must have at least 2 rows");
        }

        // Get the first and second rows
        Eigen::RowVectorXd row_x = matrix.row(0); // First row (x)
        Eigen::RowVectorXd row_y = matrix.row(1); // Second row (y)

        // Calculate min and max for both rows
        float min_x = static_cast<float>(row_x.minCoeff());
        float max_x = static_cast<float>(row_x.maxCoeff());
        float min_y = static_cast<float>(row_y.minCoeff());
        float max_y = static_cast<float>(row_y.maxCoeff());

        // Store in std::vector<float>
        return {min_x, max_x, min_y, max_y};
    }

    geometry_msgs::msg::TransformStamped getter_transform(){
//        std::cout << "t: " << t.header.frame_id << std::endl;
        return t;
    }

    bool check_person_at_loc(std::string& lndmrk){
        // bounds = [x_min, x_max, y_min, y_max]
        std::vector<float> bounds= mesh_vert_map_[lndmrk];

        geometry_msgs::msg::TransformStamped patient_location;
        try {

            patient_location = tf_buffer_->lookupTransform("unity", "nathan", tf2::TimePointZero, std::chrono::seconds(100)); //TODO fix
            float patient_x = patient_location.transform.translation.x;
            float patient_y = patient_location.transform.translation.y;
            if (bounds[0] < patient_x && patient_x < bounds[1] &&
                bounds[2] < patient_y && patient_y < bounds[3]) {
                return true;
            }
            std::cout << "above " << is_above(patient_x, bounds[1]) << std::endl;
            std::cout << "below " << is_below(patient_x, bounds[0]) << std::endl;
            std::cout <<  "is_on_left " << is_on_left(patient_y, bounds[3]) << std::endl;
            std::cout <<  "is_on_right " <<is_on_right(patient_y, bounds[2]) << std::endl;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s:", "unity", ex.what());
            return false;
        }

        return false;
    }

    bool is_above(float person_x, float x_max){
        return person_x > x_max;
    }

    bool is_below(float person_x, float x_min){
        return person_x < x_min;
    }

    bool is_on_left(float person_y, float y_max){
        return person_y > y_max;
    }

    bool is_on_right(float person_y, float y_min){
        return person_y < y_min;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MeshNode>();
    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped t;
    std::string current_state = "";
    std::string prev_state = "";
    auto tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    std::vector<std::pair<float, float>> particles;

    geometry_msgs::msg::TransformStamped t_;
    t_.header.stamp = rclcpp::Clock().now();
    t_.header.frame_id = "unity";
    t_.child_frame_id = "map";
    t_.transform.translation.x = -0.005730870385952103;
    t_.transform.translation.y = 0.04901200331852795;
    t_.transform.translation.z = -1.1970026016161086;
    t_.transform.rotation.x = 2.6707802908482766e-06;
    t_.transform.rotation.y = -3.6824548658441327e-06;
    t_.transform.rotation.z = 0.008302306210421214;
    t_.transform.rotation.w = 0.999965535251538;

    tf_static_broadcaster_->sendTransform(t_);

    std::vector<std::string>
            lndmarks = {"indoor"};
//            lndmarks = {"indoor", "outdoor"};

//    std::vector<float> bounds_outside = node->mesh_vert_map_["outdoor"];
//    for (auto i:bounds_outside){
//        std::cout << "i" << i<< std::endl;
//    }


//    std::random_device rd;  // Seed
//    std::mt19937 gen(rd()); // Random number generator
//    std::uniform_real_distribution<float> dist_x(bounds_outside[0], bounds_outside[1]);
//    std::uniform_real_distribution<float> dist_y(bounds_outside[2], bounds_outside[3]);
//    std::cout << "GEN: " << std::endl;

//    for (int i = 0; i < 200; ++i) {
////                    std::cout << "current_state: " << current_state << " prev_state: " << prev_state << std::endl;
//        std::pair<int, int> p;
//        // Generate random values within the bounds
//        p.first = static_cast<int>(dist_x(gen));
//        p.second = static_cast<int>(dist_y(gen));
//        particles.push_back(p);
//
//    }
//    std::cout << "PUBLISH: " << std::endl;
//    node->publish_particles(particles);

    while (rclcpp::ok()) {

        t = node->getter_transform();

        if (!t.header.frame_id.empty()){
            t.header.stamp = rclcpp::Clock().now();
            tf_broadcaster_->sendTransform(t);


            for(std::string lndmrk : lndmarks) {
//                std::cout << "lndmrk: " << lndmrk << std::endl;
                if (node->check_person_at_loc(lndmrk)){
                    std::cout << "PERSON  AT " << lndmrk << std::endl;

                    current_state = lndmrk;
                    std::cout << "current_state: " << current_state << std::endl;
                }else{
                    current_state = "";
                }
            }
            std::cout << "current_state: " << current_state << " prev_state: " << prev_state << std::endl;
            if (current_state!="" && prev_state!="" && current_state == prev_state){
                std::cout << "bounds_outside: " << std::endl;
                std::vector<float> bounds_outside = node->mesh_vert_map_["outdoor"];

                std::random_device rd;  // Seed
                std::mt19937 gen(rd()); // Random number generator
                std::uniform_real_distribution<float> dist_x(bounds_outside[0], bounds_outside[1]);
                std::uniform_real_distribution<float> dist_y(bounds_outside[2], bounds_outside[3]);
                std::cout << "GEN: " << std::endl;

                for (int i = 0; i < 200; ++i) {
//                    std::cout << "current_state: " << current_state << " prev_state: " << prev_state << std::endl;
                    std::pair<float, float> p;
                    // Generate random values within the bounds
                    p.first = static_cast<float>(dist_x(gen));
                    p.second = static_cast<float>(dist_y(gen));
                    std::cout << "p.first : " << p.first << "p.second : " << p.second << std::endl;
                    particles.push_back(p);

                }
                std::cout << "PUBLISH: " << std::endl;
                node->publish_particles(particles);

            }
        }

        prev_state = current_state;
        rclcpp::spin_some(node);
    }

//    thread_1.join();
    rclcpp::shutdown();

    return 0;
}




//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

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


#include <random>
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
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped t;
    std::unordered_map<std::string, std::vector<float> > mesh_vert_map_;

public:
    MeshNode() : Node("particle_filter") {
        clicked_point = create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point", 1,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { ClickedPointCallback(msg); });

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


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

    auto tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

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

    while (rclcpp::ok()) {

        t = node->getter_transform();

        if (!t.header.frame_id.empty()){
            t.header.stamp = rclcpp::Clock().now();
            tf_broadcaster_->sendTransform(t);


            for(std::string lndmrk : lndmarks) {
//                std::cout << "lndmrk: " << lndmrk << std::endl;
                if (node->check_person_at_loc(lndmrk)){
                    std::cout << "PERSON  AT " << lndmrk << std::endl;
                }
            }
        }
        rclcpp::spin_some(node);
    }

//    thread_1.join();
    rclcpp::shutdown();

    return 0;
}




//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

//#include <sensor_msgs/msg/camera_info.hpp>
//#include <sensor_msgs/msg/image.hpp>
//#include <geometry_msgs/msg/point.hpp>
//#include <geometry_msgs/msg/vector3.hpp>
//#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <geometry_msgs/msg/pose.hpp>

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
#include "particle_filter_msgs/msg/door_status.hpp"
#include <array>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "zed_interfaces/msg/objects_stamped.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "particle_filter_msgs/msg/pose_msg.hpp"
#include "zed_interfaces/msg/bounding_box3_d.hpp"
#include "zed_interfaces/msg/object.hpp"
#include <cstdlib>


class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_3d_pt;

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;
    rclcpp::Subscription<particle_filter_msgs::msg::PoseMsg>::SharedPtr pose_sub_k;
    rclcpp::Subscription<particle_filter_msgs::msg::PoseMsg>::SharedPtr pose_sub_dn;
    rclcpp::Subscription<particle_filter_msgs::msg::PoseMsg>::SharedPtr pose_sub_dw;
    Observation observation; // Member variable to store the observation
    // to prevent overriding
    Observation observation_kitchen; // Member variable to store the observation from kitchen
    Observation observation_dining; // Member variable to store the observation from dining
    Observation observation_doorway; // Member variable to store the observation from doorway

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::map<std::string, std::string> map_cam_aptag;
    std::map<std::string, std::string> map_cam_aptag_un;

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_bedroom;
    bool door_bathroom;


public:
    ParticleFilterNode() : Node("particle_filter") {

        map_cam_aptag["doorway"] = "tag_" + std::string(std::getenv("tag_doorway")) + "_zed";
        map_cam_aptag["kitchen"] = "tag_" + std::string(std::getenv("tag_kitchen")) + "_zed";
        //map_cam_aptag["dining_room"] = "tag_" + std::string(std::getenv("tag_dining_room")) + "_zed";

        map_cam_aptag_un["doorway"] = "aptag_" + std::string(std::getenv("tag_doorway"));
        map_cam_aptag_un["kitchen"] = "aptag_" + std::string(std::getenv("tag_kitchen"));
        //map_cam_aptag_un["dining_room"] = "aptag_" + std::string(std::getenv("tag_dining_room"));

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_sub_k = create_subscription<particle_filter_msgs::msg::PoseMsg>(
                "/kitchen_person_pose", 1,
                [this](const particle_filter_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_kitchen(msg); });

//        pose_sub_dn = create_subscription<particle_filter_msgs::msg::PoseMsg>(
//                "/dining_room_person_pose", 1,
//                [this](const particle_filter_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_dining_room(msg); });

        pose_sub_dw = create_subscription<particle_filter_msgs::msg::PoseMsg>(
                "/doorway_person_pose", 1,
                [this](const particle_filter_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_doorway(msg); });


        auto door_outdoor_sub = create_subscription<particle_filter_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_outdoor", 10,
                [this](const particle_filter_msgs::msg::DoorStatus::SharedPtr msg) { DoorOutdoorCallback(msg); });
        auto door_bedroom_sub = create_subscription<particle_filter_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_bedroom", 10,
                [this](const particle_filter_msgs::msg::DoorStatus::SharedPtr msg) { DoorBedroomCallback(msg); });
        auto door_bathroom_sub = create_subscription<particle_filter_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_bathroom", 10,
                [this](const particle_filter_msgs::msg::DoorStatus::SharedPtr msg) { DoorBathroomCallback(msg); });
    }

    std::array<double, 4> sigma_pos;

    void DoorOutdoorCallback(const particle_filter_msgs::msg::DoorStatus::SharedPtr &msg) {
        std::cout << " ######################################################" << std::endl;
        door_outdoor = msg->open;
        std::cout << "msg->open;" << msg->open << std::endl;
        std::cout << "doorstats->open;" << door_outdoor << std::endl;
    }

    void DoorBedroomCallback(const particle_filter_msgs::msg::DoorStatus::SharedPtr &msg) {
        std::cout << "********************************" << std::endl;
        door_bedroom = msg->open;
        std::cout << "bedroom msg->open;" << msg->open << std::endl;
        std::cout << "bedoroom doorstats->open;" << door_bedroom << std::endl;
    }

    void DoorBathroomCallback(const particle_filter_msgs::msg::DoorStatus::SharedPtr &msg) {
        std::cout << "9999999999999999999999999999999999999" << std::endl;
        door_bathroom = msg->open;
        std::cout << "bsth msg->open;" << msg->open << std::endl;
        std::cout << "bedbathoroom doorstats->open;" << door_bathroom << std::endl;
    }

    std::vector<bool> getdoorstatus() {
        // should align with patrticle filter enforce collision landmarks orderc
//        bedroom_door, bathroom_door, living_room_door, outside_door
        return {door_bedroom, door_bathroom, door_outdoor};
    }

    Observation getObservation() {
        if (observation_kitchen.name != ""){
            std::cout << "observation in kitchen" << observation_kitchen.name  << std::endl;
            return observation = observation_kitchen;
        }
//        if (observation_dining.name != ""){
//            return observation = observation_dining;
//        }
        else if(observation_doorway.name != ""){
            std::cout << "observation in doorway " << observation_doorway.name << std::endl;
            return observation = observation_doorway;
        }
        else{
            observation.name = "";
            return observation;
        }
    }

    void PosePixCallback_kitchen(const particle_filter_msgs::msg::PoseMsg::SharedPtr &msg) {
        //# 2 -> POSE_38
        if (msg != nullptr && !(msg->label == "") ) {
                observation_kitchen.name = "kitchen";
                zed_interfaces::msg::BoundingBox3D bounding_box = msg->bounding_box;
                float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
                for (int i = 0; i < 8; i++) {
                    sum_x += bounding_box.corners[i].kp[0];
                    sum_y += bounding_box.corners[i].kp[1];
                    sum_z += bounding_box.corners[i].kp[2];
                }

                // Calculate the centroid
                observation_kitchen.x = sum_x / 8.0;
                observation_kitchen.y = sum_y / 8.0;
                observation_kitchen.z = sum_z / 8.0;

                sigma_pos[0] = msg->dimensions_3d.data[0];
                sigma_pos[1] = msg->dimensions_3d.data[1];
                sigma_pos[2] = msg->dimensions_3d.data[2];
                sigma_pos[3] = 0.1;

        } else {
            std::cout << "no person detected" << std::endl;
            observation_kitchen.name = "";

        }
    }

    void PosePixCallback_doorway(const particle_filter_msgs::msg::PoseMsg::SharedPtr &msg) {
        //# 2 -> POSE_38
        if (!(msg->label == "")) {
                observation_doorway.name = "doorway";
                zed_interfaces::msg::BoundingBox3D bounding_box = msg->bounding_box;
                float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
                for (int i = 0; i < 8; i++) {
                    sum_x += bounding_box.corners[i].kp[0];
                    sum_y += bounding_box.corners[i].kp[1];
                    sum_z += bounding_box.corners[i].kp[2];
                }

                // Calculate the centroid
                observation_doorway.x = sum_x / 8.0;
                observation_doorway.y = sum_y / 8.0;
                observation_doorway.z = sum_z / 8.0;

                sigma_pos[0] = msg->dimensions_3d.data[0];
                sigma_pos[1] = msg->dimensions_3d.data[1];
                sigma_pos[2] = msg->dimensions_3d.data[2];
                sigma_pos[3] = 0.1;

        } else {
            std::cout << "no person detected" << std::endl;
            observation_doorway.name = "";

        }
    }

//    void PosePixCallback_dining_room(const particle_filter_msgs::msg::PoseMsg::SharedPtr &msg) {
//        //# 2 -> POSE_38
//        if (!(msg->label == "")) {
//                observation_dining.name = "dining_room";
//                zed_interfaces::msg::BoundingBox3D bounding_box = msg->bounding_box;
//                float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
//                for (int i = 0; i < 8; i++) {
//                    sum_x += bounding_box.corners[i].kp[0];
//                    sum_y += bounding_box.corners[i].kp[1];
//                    sum_z += bounding_box.corners[i].kp[2];
//                }
//
//                // Calculate the centroid
//                observation_dining.x = sum_x / 8.0;
//                observation_dining.y = sum_y / 8.0;
//                observation_dining.z = sum_z / 8.0;
//
//                sigma_pos[0] = msg->dimensions_3d.data[0];
//                sigma_pos[1] = msg->dimensions_3d.data[1];
//                sigma_pos[2] = msg->dimensions_3d.data[2];
//                sigma_pos[3] = 0.1;
//
//        } else {
//            std::cout << "no person detected" << std::endl;
//            observation_dining.name = "";
//
//        }
//    }


    void publish_3d_point(float x, float y, float z, std::string frame_id, float r, float g, float b) {
        auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        // Create a marker message for a single point
//        visualization_msgs::msg::Marker marker_msg;
        marker_msg->header.frame_id = frame_id; // Replace with your desired frame_id
        marker_msg->header.stamp = this->get_clock()->now();
        marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg->action = visualization_msgs::msg::Marker::ADD;
        marker_msg->scale.x = 0.1; // Scale factors for the point (adjust as needed)
        marker_msg->scale.y = 0.1;
        marker_msg->scale.z = 0.1;
        marker_msg->color.r = r;
        marker_msg->color.g = g;
        marker_msg->color.b = b;
        marker_msg->color.a = 1.0;

        // Set the position of the point
        marker_msg->pose.position.x = x; // Replace with your desired X coordinate
        marker_msg->pose.position.y = y; // Replace with your desired Y coordinate
        marker_msg->pose.position.z = z; // Replace with your desired Z coordinate

        // Publish the marker
//        publisher_3d_pt->publish(*marker_msg);
    }

    void publish_particles(std::vector<Particle> &particles) {       // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        // Populate the marker array with markers
        for (const auto &particle: particles) {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "unity";
            marker.header.stamp = this->get_clock()->now();
            marker.id = particle.id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = particle.x;
            marker.pose.position.y = particle.y;
            marker.pose.position.z = particle.z;
            marker.pose.orientation.z = sin(particle.theta / 2.0);
            marker.pose.orientation.w = cos(particle.theta / 2.0);
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

    void cam_extrinsics_from_tf() {

//        std::vector<std::string> cams{"dining", "kitchen", "bedroom", "livingroom", "hallway", "doorway"};
        std::vector<std::string> cams{"kitchen", "doorway"};
//        std::vector<std::pair<std::string, int>> cams{"zed_kitchen_left_camera_frame"};

        // Loop over the keys of map_cam_aptag using a range-based for loop
        for (const auto &cam: cams) {
            // Get transformation matrix from camera to aptag /// from aptag detection
            std::cout << " cAM_cam " << cam << std::endl;
            std::cout << "  " << map_cam_aptag[cam] << "  " << "zed_" + cam + "_left_camera_frame" << std::endl;
//
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_cam_to_aptag = transform_tf(map_cam_aptag[cam],
                                                                                       "zed_" + cam +
                                                                                       "_left_camera_frame");
            std::cout << " t_cam_to_aptag " << t_cam_to_aptag << std::endl;
//
            // Get transformation matrix from map to waptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_waptag_to_cam = transform_tf("unity", map_cam_aptag_un[cam]);
            std::cout << " t_waptag_to_cam " << t_waptag_to_cam << std::endl;
//
//            // Get transformation matrix from map to aptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_cam_to_map = t_waptag_to_cam * t_cam_to_aptag;
            std::cout << " t_cam_to_map " << t_cam_to_map << std::endl;
//
            cameraextrinsics.insert(std::make_pair(cam, t_cam_to_map));
        }
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_tf(std::string toFrame, std::string fromFrame) {
        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(), std::chrono::milliseconds(100000));

            geometry_msgs::msg::Transform transform_ = t.transform;

            // turn geometry transform to 4x4 matrix
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform = transform_geometry_to_matrix(transform_);
            RCLCPP_INFO(this->get_logger(), "transform %s to %s", fromFrame.c_str(), toFrame.c_str());

            return transform;

        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    fromFrame.c_str(), toFrame.c_str(), ex.what());
//            return;
        }
    }

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_cam_extrinsic_matrix() {
        cam_extrinsics_from_tf();
        return cameraextrinsics;
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_geometry_to_matrix(geometry_msgs::msg::Transform transform) {
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicmatrix;
        Eigen::Quaterniond quaternion(transform.rotation.w,
                                      transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z);
        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();

        Eigen::Vector3d translationVector(transform.translation.x,
                                          transform.translation.y,
                                          transform.translation.z);

        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
        extrinsicmatrix.row(3) << 0, 0, 0, 1;
        return extrinsicmatrix;
    }

    geometry_msgs::msg::TransformStamped
    publish_transform(Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transformation_matrix, std::string frame_id,
                      std::string child_frame_id) {

        Eigen::Affine3d affine(transformation_matrix);
        Eigen::Quaterniond quaternion(affine.linear());
        Eigen::Vector3d translation(affine.translation());

        // Fill in the message
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = frame_id;
        t.child_frame_id = child_frame_id;
        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();
        t.transform.rotation.x = quaternion.x();
        t.transform.rotation.y = quaternion.y();
        t.transform.rotation.z = quaternion.z();
        t.transform.rotation.w = quaternion.w();
        return t;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParticleFilterNode>();

    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    auto tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> camera_extrinsics;

    // Todo map observation to camera intrinsic and extrinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));

    bool not_initialized = true;
    while (rclcpp::ok()) {
        /// comment when not debugging
        if (not_initialized) {
            camera_extrinsics = node->get_cam_extrinsic_matrix();

            if (camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {

            std::array<double, 4> sigma_pos = {0.3, 0.3, 0.3, 0.01};

            double sigma_landmark[3] = {0.04, 0.04, 0.04};

            // noise generation
            std::default_random_engine gen;

            std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
            std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);

            double n_x, n_y;

            // Define the bounds based on the house
            //std::pair<double, double> x_bound = std::make_pair(-1.0, 1.0);
            //std::pair<double, double> y_bound = std::make_pair(-1.0, 1.0);
            
            std::pair<double, double> x_bound = std::make_pair(-3.0, 2.0);
            std::pair<double, double> y_bound = std::make_pair(-5.0, 1.0);
            
            std::pair<double, double> z_bound = std::make_pair(0, 0);
            std::pair<double, double> theta_bound = std::make_pair(-3.1416, 3.1416);

            int num_particles = 500; // has to be multiple of 128

            double velocity = 0.01;
            double yaw_rate = 0.5;
            bool running = true;

            ParticleFilter particle_filter(num_particles);

            for (const auto &entry: camera_extrinsics) {
                const std::string &camera_name = entry.first;
                const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &extrinsic_matrix = entry.second;

                auto t_ = node->publish_transform(extrinsic_matrix, "unity", "zed_cam" + camera_name);

                tf_static_broadcaster_->sendTransform(t_);
            }

            while (running) {
//                auto beg = std::chrono::high_resolution_clock::now();

                // bedroom_door, bathroom_door, living_room_door, outside_door
//                std::vector<bool> door_status_ = {0,0,0,0};
                std::vector<bool> door_status_ = node->getdoorstatus();
                if (!particle_filter.initialized()) {
                    // Initialize the particle filter in a uniform distribution
                    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
                    node->publish_particles(particle_filter.particles);

                } else {
                    /// not being used delta_t
//                    auto end = std::chrono::high_resolution_clock::now();
//                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
//                    double delta_t = duration.count() / 1000000.0;
                    double delta_t = 0.1;

                    particle_filter.motion_model(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_);
                    node->publish_particles(particle_filter.particles);
                }


                // get observation and skip if no observation is there
                std::vector<Observation> observations;
                Observation obs_ = node->getObservation();
                if (obs_.name != "") {
                    Eigen::Vector4d homogeneousPoint;
                    homogeneousPoint << obs_.x, obs_.y, obs_.z, 1.0;
//                    node->publish_3d_point(homogeneousPoint[0], homogeneousPoint[1], homogeneousPoint[2], "zed_cam", 1,
//                                           0, 0);

                    std::string cam_name = obs_.name;
                    std::cout << "  cam_name  " << cam_name << std::endl;
                    auto extrinsicParams = camera_extrinsics[cam_name];

                    Eigen::Vector4d TransformedPoint;
                    TransformedPoint << extrinsicParams(0, 0) * homogeneousPoint[0] +
                                        extrinsicParams(0, 1) * homogeneousPoint[1] +
                                        extrinsicParams(0, 2) * homogeneousPoint[2] +
                                        extrinsicParams(0, 3) * homogeneousPoint[3],
                            extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
                            extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
                            extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
                            extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
                            extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
                            extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];

                    std::cout << "  camera_extrinsics[obs_.name]  " << extrinsicParams << std::endl;
                    //node->publish_3d_point(TransformedPoint[0], TransformedPoint[1], TransformedPoint[2], "map", 0, 0,
//                                           1);

                    // observation will always be from the same camera
                    observations.push_back(obs_);

                    // simulate the addition of noise to noiseless observation data.
                    std::vector<Observation> noisy_observations;
                    Observation obs;

                    // which is currently 1
                    for (int j = 0; j < observations.size(); ++j) {
                        n_x = N_obs_x(gen);
                        n_y = N_obs_y(gen);
                        obs = observations[j];
                        obs.x = obs.x + n_x;
                        obs.y = obs.y + n_y;
                        noisy_observations.push_back(obs);
                    }

                    // Update the weights and resample
                    particle_filter.updateWeights(sigma_landmark, noisy_observations,
                                                  extrinsicParams);
                    particle_filter.resample();
                }
//                node->publish_particles(particle_filter.particles);
//
                // Calculate and output the average weighted error of the particle filter over all time steps so far.
                std::vector<Particle> particles = particle_filter.particles;
                int num_particles = particles.size();
                double highest_weight = 0.0;

                Particle best_particle;
                for (int i = 0; i < num_particles; ++i) {
                    if (particles[i].weight > highest_weight) {
                        highest_weight = particles[i].weight;
                        best_particle = particles[i];
                    }
                }

                // Fill in the message
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = rclcpp::Clock().now();
                t.header.frame_id = "unity";
                /// should be whatever the code is expecting the name to be
                t.child_frame_id = "nathan";
                t.transform.translation.x = best_particle.x;
                t.transform.translation.y = best_particle.y;
                t.transform.translation.z = best_particle.z;
                t.transform.rotation.x = 0;
                t.transform.rotation.y = 0;
                t.transform.rotation.z = sin(best_particle.theta / 2.0);
                t.transform.rotation.w = cos(best_particle.theta / 2.0);
//                std::cout << " x " << best_particle.x << " y " << best_particle.y << " z " << best_particle.z << std::endl;
                tf_broadcaster_->sendTransform(t);

                // because we want to listen to observations in this loop as well so we need to spin the node
                rclcpp::spin_some(node);
            }

        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}

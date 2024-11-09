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
#include <geometry_msgs/msg/point.hpp>
//#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/bool.hpp"


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
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub_k;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub_lv;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub_dw;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub_cor;
    Observation observation; // Member variable to store the observation
    // to prevent overriding
    Observation observation_kitchen; // Member variable to store the observation from kitchen
    Observation observation_living; // Member variable to store the observation from dining
    Observation observation_doorway; // Member variable to store the observation from doorway
    Observation observation_corridor; // Member variable to store the observation from corridor

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::map<std::string, std::string> map_cam_aptag;
//    std::map<std::string, std::string> map_cam_aptag_un;

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_bedroom;
    bool door_bathroom;


public:
    ParticleFilterNode() : Node("particle_filter") {

//        map_cam_aptag["doorway"] = "tag_" + std::string(std::getenv("tag_doorway")) + "_zed";
//        map_cam_aptag["kitchen"] = "tag_" + std::string(std::getenv("tag_kitchen")) + "_zed";
//        map_cam_aptag["dining_room"] = "tag_" + std::string(std::getenv("tag_dining_room")) + "_zed";
//
//        map_cam_aptag_un["doorway"] = "aptag_" + std::string(std::getenv("tag_doorway"));
//        map_cam_aptag_un["kitchen"] = "aptag_" + std::string(std::getenv("tag_kitchen"));
//        map_cam_aptag_un["dining_room"] = "aptag_" + std::string(std::getenv("tag_dining_room"));

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_sub_k = create_subscription<geometry_msgs::msg::Point>(
                "/zed_kitchen", 1,
                [this](const geometry_msgs::msg::Point::SharedPtr msg) { PosePixCallback_kitchen(msg); });

        pose_sub_lv = create_subscription<geometry_msgs::msg::Point>(
                "/zed_living_room", 1,
                [this](const geometry_msgs::msg::Point::SharedPtr msg) { PosePixCallback_living_room(msg); });

        pose_sub_dw = create_subscription<geometry_msgs::msg::Point>(
                "/zed_doorway", 1,
                [this](const geometry_msgs::msg::Point::SharedPtr msg) { PosePixCallback_doorway(msg); });

        pose_sub_cor = create_subscription<geometry_msgs::msg::Point>(
                "/zed_corridor", 1,
                [this](const geometry_msgs::msg::Point::SharedPtr msg) { PosePixCallback_corridor(msg); });


        auto door_outdoor_sub = create_subscription<std_msgs::msg::Bool>(
                "/smartthings_sensors_door_outdoor", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorOutdoorCallback(msg); });
        auto door_bedroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/smartthings_sensors_door_bedroom", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBedroomCallback(msg); });
        auto door_bathroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/smartthings_sensors_door_bathroom", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBathroomCallback(msg); });
    }

    // save coordinate map
    const std::unordered_map<std::string, std::tuple<double, double, double>> coordinate_map = {
            {"living_room", {0.85, -0.31, 0.0}},  // x, y, z coordinates
            {"bedroom",     {-5.1, -1.2, 0.0}},
            {"outside",     {6, -0.7, 0.0}},
    };

    std::array<double, 4> sigma_pos;

    void DoorOutdoorCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
        std::cout << " ######################################################" << std::endl;
        door_outdoor = msg->data;
        std::cout << "msg->open;" << msg->data << std::endl;
        std::cout << "doorstats->open;" << door_outdoor << std::endl;
    }

    void DoorBedroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
        std::cout << "********************************" << std::endl;
        door_bedroom = msg->data;
        std::cout << "bedroom msg->open;" << msg->data << std::endl;
        std::cout << "bedoroom doorstats->open;" << door_bedroom << std::endl;
    }

    void DoorBathroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
        std::cout << "9999999999999999999999999999999999999" << std::endl;
        door_bathroom = msg->data;
        std::cout << "bsth msg->open;" << msg->data << std::endl;
        std::cout << "bedbathoroom doorstats->open;" << door_bathroom << std::endl;
    }

    std::vector<bool> getdoorstatus() {
        // should align with patrticle filter enforce collision landmarks orderc
//        bedroom_door, bathroom_door, living_room_door, outside_door
        return {door_bedroom, door_bathroom, door_outdoor};
    }

    Observation getObservation() {
        float distance_to_person = 100.0;
        std::string name = "";

        if (observation_kitchen.name != "") {
            distance_to_person = observation_kitchen.x;
            std::cout << "observation in kitchen" << observation_kitchen.name << std::endl;
            observation = observation_kitchen;
        }

        if (observation_doorway.name != "") {
            if (distance_to_person > observation_doorway.x) {
                distance_to_person = observation_doorway.x;
                observation = observation_doorway;
            }
            std::cout << "observation in doorway " << observation_doorway.name << std::endl;
//            return observation = observation_doorway;
        }
        if (observation_living.name != "") {
            if (distance_to_person > observation_living.x) {
                distance_to_person = observation_living.x;
                observation = observation_living;
                std::cout << "observation in living " << observation_doorway.name << std::endl;
            }

        }
        if (observation_corridor.name != "") {
            if (distance_to_person > observation_corridor.x) {
                distance_to_person = observation_corridor.x;
                observation = observation_corridor;
                std::cout << "observation in corridor " << observation_doorway.name << std::endl;
            }

        }

        if (distance_to_person == 100.0) {
            observation.name = "";
        }
        std::cout << "observation in " << observation.name << std::endl;
        return observation;

    }

    void PosePixCallback_kitchen(const geometry_msgs::msg::Point::SharedPtr &msg) {
        //# 2 -> POSE_38
        std::cout << " ************** Person detected in kitchen" << std::endl;
        if(msg){
            observation_kitchen.name = "kitchen";
            // Calculate the centroid
            observation_kitchen.x = msg->x;
            observation_kitchen.y = msg->y;
            observation_kitchen.z = msg->z;

            sigma_pos[0] = 0.1;
            sigma_pos[1] = 0.1;
            sigma_pos[2] = 0.1;
            sigma_pos[3] = 0.1;

        } else {
            std::cout << "no person detected in kitchen" << std::endl;
            observation_kitchen.name = "";

        }
    }

    void PosePixCallback_doorway(const geometry_msgs::msg::Point::SharedPtr &msg) {
        //# 2 -> POSE_38
        if (msg->z == 0.0) {
            observation_doorway.name = "doorway";

            // Calculate the centroid
            observation_doorway.x = msg->x;
            observation_doorway.y = msg->y;
            observation_doorway.z = msg->z;

            sigma_pos[0] = 0.1;
            sigma_pos[1] = 0.1;
            sigma_pos[2] = 0.1;
            sigma_pos[3] = 0.1;

        } else {
            std::cout << "no person detected in doorway" << std::endl;
            observation_doorway.name = "";

        }
    }

    void PosePixCallback_corridor(const geometry_msgs::msg::Point::SharedPtr &msg) {
        //# 2 -> POSE_38
        if (msg->z == 0.0) {
            observation_corridor.name = "corridor";

            // Calculate the centroid
            observation_corridor.x = msg->x;
            observation_corridor.y = msg->y;
            observation_corridor.z = msg->z;

            sigma_pos[0] = 0.1;
            sigma_pos[1] = 0.1;
            sigma_pos[2] = 0.1;
            sigma_pos[3] = 0.1;

        } else {
            std::cout << "no person detected in doorway" << std::endl;
            observation_corridor.name = "";

        }
    }

    void PosePixCallback_living_room(const geometry_msgs::msg::Point::SharedPtr &msg) {
        if (msg->z == 0.0) {
            observation_living.name = "living_room";

            // Calculate the centroid
            observation_living.x = msg->x;
            observation_living.y = msg->y;
            observation_living.z = msg->z;

            sigma_pos[0] = 0.1;
            sigma_pos[1] = 0.1;
            sigma_pos[2] = 0.1;
            sigma_pos[3] = 0.1;
        } else {
            std::cout << "no person detected in doorway" << std::endl;
            observation_living.name = "";

        }
    }

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
//                for (const auto &entry: camera_extrinsics) {
//                    const std::string &camera_name = entry.first;
//                    const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &extrinsic_matrix = entry.second;
//                    auto t_ = node->publish_transform(extrinsic_matrix, "unity", "zed_cam_" + camera_name);
//                    tf_static_broadcaster_->sendTransform(t_);
//                }

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

            std::pair<double, double> x_bound = std::make_pair(-1.0, 1.0);
            std::pair<double, double> y_bound = std::make_pair(-1.0, 1.0);

            std::pair<double, double> z_bound = std::make_pair(0, 0);
            std::pair<double, double> theta_bound = std::make_pair(-3.1416, 3.1416);

            int num_particles = 500;

            double velocity = 0.01;
            double yaw_rate = 0.5;
            bool running = true;

            ParticleFilter particle_filter(num_particles);

            for (const auto &entry: camera_extrinsics) {
                const std::string &camera_name = entry.first;
                const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &extrinsic_matrix = entry.second;

                auto t_ = node->publish_transform(extrinsic_matrix, "unity", "zed_cam_" + camera_name);
                tf_static_broadcaster_->sendTransform(t_);
            }
            int count_empty = 0;
            int count_threshold = 5;
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

                    // get observation and skip if no observation is there
                    std::vector<Observation> observations;
                    Observation obs_ = node->getObservation();

                    particle_filter.curr_camera_name = obs_.name;
//                    if (particle_filter.curr_camera_name != particle_filter.prev_camera_name &&
//                        !particle_filter.curr_camera_name.empty()) {
//                        // have current observation with NAN cause no observation
//                        particle_filter.current_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
//                    }

                    // have current observation with NAN cause no observation
                    if (particle_filter.curr_camera_name.empty()) {
                        particle_filter.current_observation = Eigen::Vector2d::Constant(
                                std::numeric_limits<double>::quiet_NaN());
                    }

                    /// not being used delta_t
//                    auto end = std::chrono::high_resolution_clock::now();
//                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
//                    double delta_t = duration.count() / 1000000.0;
                    double delta_t = 0.1;

                    particle_filter.motion_model(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_, obs_.name);
                    node->publish_particles(particle_filter.particles);


                    if (!obs_.name.empty()) {
                        //particle_filter.previous_observation.push_back(Eigen::Vector2d(obs_.x, obs_.y));
                        Eigen::Vector4d homogeneousPoint;
                        homogeneousPoint << obs_.x, obs_.y, obs_.z, 1.0;
//                    node->publish_3d_point(homogeneousPoint[0], homogeneousPoint[1], homogeneousPoint[2], "zed_cam", 1,
//                                           0, 0);

                        std::string cam_name = obs_.name;
                        std::cout << "  cam_name  " << cam_name << std::endl;
                        auto extrinsicParams = camera_extrinsics[cam_name];

                        // observation will always be from the same camera
                        observations.push_back(obs_);

                        // simulate the addition of noise to noiseless observation data.
                        std::vector<Observation> noisy_observations;
                        Observation obs;

                        // which is currently 1
                        for (int j = 0; j < observations.size(); ++j) {
//                            n_x = N_obs_x(gen);
//                            n_y = N_obs_y(gen);
                            obs = observations[j];
//                            obs.x = obs.x + n_x;
//                            obs.y = obs.y + n_y;
                            noisy_observations.push_back(obs);
                        }

                        // Update the weights and resample
                        particle_filter.updateWeights(sigma_landmark, noisy_observations,
                                                      extrinsicParams);
                        particle_filter.resample();

                        // node->publish_particles(particle_filter.particles);


                        particle_filter.prev_camera_name = particle_filter.curr_camera_name;

                    }
                    std::vector<Particle> particles = particle_filter.particles;
                    int num_particles = particles.size();
                    double highest_weight = 0.0;

                    Particle best_particle;

                    // Fill in the message
                    geometry_msgs::msg::TransformStamped t;
                    /// should be whatever the code is expecting the name to be
                    t.child_frame_id = "nathan";

                    if (!particle_filter.use_max_loc) {
                        for (int i = 0; i < num_particles; ++i) {
                            if (particles[i].weight > highest_weight) {
                                highest_weight = particles[i].weight;
                                best_particle = particles[i];
                            }
                        }
                        t.header.frame_id = "unity";
                        t.transform.translation.x = best_particle.x;
                        t.transform.translation.y = best_particle.y;
                        t.transform.translation.z = best_particle.z;
                        t.transform.rotation.x = 0;
                        t.transform.rotation.y = 0;
                        t.transform.rotation.z = sin(best_particle.theta / 2.0);
                        t.transform.rotation.w = cos(best_particle.theta / 2.0);
                        // std::cout << " x " << best_particle.x << " y " << best_particle.y << " z " << best_particle.z << std::endl;
                        // t = node -> compute_mean_point(particle_filter.particles);

                    } else {
                        auto it = node->coordinate_map.find(particle_filter.max_particles_loc);
                        std::cout << "max_loc _ " <<  particle_filter.max_particles_loc << std::endl;

                        if (it != node->coordinate_map.end()) {
                            t.transform.translation.x = std::get<0>(it->second);
                            t.transform.translation.y = std::get<1>(it->second);
                            t.transform.translation.z = std::get<2>(it->second);
                        } else {
                            // Handle the case where the landmark is not found in the map
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Landmark %s not found in the map!", particle_filter.max_particles_loc.c_str());
                        }

                        t.transform.rotation.x = 0;
                        t.transform.rotation.y = 0;
                        t.transform.rotation.z = 0;
                        t.transform.rotation.w = 1;
                        t.header.frame_id = "map";

                    }
                    t.header.stamp = rclcpp::Clock().now();
                    tf_broadcaster_->sendTransform(t);



                    // because we want to listen to observations in this loop as well so we need to spin the node
                    rclcpp::spin_some(node);
                }
            }
        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}

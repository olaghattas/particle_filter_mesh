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
#include <particle_filter/particle_filter_node.hpp>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    auto tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    std::map <std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> camera_extrinsics;
    bool first_obs = false;


    std::pair<double, double> x_bound = std::make_pair(-1.0, 1.0);
    std::pair<double, double> y_bound = std::make_pair(-1.0, 1.0);

    std::pair<double, double> z_bound = std::make_pair(0, 0);
    std::pair<double, double> theta_bound = std::make_pair(-3.1416, 3.1416);
    Observation obs_;
    int num_particles = 500;
    ParticleFilter particle_filter(num_particles);

    bool not_initialized = true;

    double delta_t = 0.1;
    double velocity = 0.01;
    double yaw_rate = 0.5;
    std::vector<bool> door_status_;
    std::vector<bool> ms_status_;
    double sigma_landmark[3] = {0.04, 0.04, 0.04};

//    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
    std::cout << " INIT : ^^^^ " << std::endl;
//    node->publish_particles(particle_filter.particles);
//    std::vector<Particle> particles = particle_filter.particles;

    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
    node->publish_particles(particle_filter.particles);

    bool getting_initial_dist = false;
    while(getting_initial_dist){
        door_status_ = {true, true, true};
        ms_status_ = node->getmsstatus();
        particle_filter.motion_model_noisy(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_, obs_.name, node->currentStateH, ms_status_);
        node->publish_particles(particle_filter.particles);
//        particle_filter.write_to_file("/home/olagh48652/particle_filter_ws/src/particle_filter_mesh/config/initial_dist_exeter.txt");
        particle_filter.find_landmark_with_most_particles();
    }

    while (rclcpp::ok()) {
//        std::cout << " RCLPY OKAY : ^^^^ " << std::endl;
        if (not_initialized) {

            std::cout << " not_initialized : ^^^^ " << std::endl;

            camera_extrinsics = node->get_cam_extrinsic_matrix();
            // make sure camera positions are set
            if (camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {

            door_status_ = node->getdoorstatus();
            ms_status_ = node->getmsstatus();

//            std::cout << "{door_bedroom, door_bathroom, door_outdoor};" << std::endl;
//            std::cout << "door_status_ close: " << door_status_[2] << std::endl;
            // Initialize the particle filter in a uniform distribution

            obs_ = node->getObservation(particle_filter);
            first_obs = node->first_obs;

            // for debug
//            first_obs = true;
            // NO FIRST OBSERVATION KEEP DISTRIBUTION AS IS
            if (first_obs) {

                // Fill in the message
                geometry_msgs::msg::TransformStamped t;
                /// should be whatever the code is expecting the name to be
                t.child_frame_id = "nathan";

                particle_filter.motion_model_noisy(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_, obs_.name, node->currentStateH, ms_status_);
//                node->publish_particles(particle_filter.particles);

                if (obs_.name.empty()) {
                    //observation empty

                    // Update the weights and resample
                    particle_filter.updateWeightsWithoutObs(sigma_landmark);
                    node->publish_particles(particle_filter.particles);

                    std::cout << " updateWeightsWithoutObs  " << std::endl;

                    double Neff = particle_filter.calculateNeff();
                    // resample if too few effective particles
                    std::cout << " Neff:  " << Neff << std::endl;
                    std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

                    if (Neff < particle_filter.num_particles / 3) {
                        std::cout << " resample  " << std::endl;

                        particle_filter.resample();
                        particle_filter.check_unique_particles();
                        node->publish_particles(particle_filter.particles);

                    }

//                  publish location  in the location with the most particles
                    auto it = node->coordinate_map.find(particle_filter.max_particles_loc);
//                    std::cout << "max_loc _ " << particle_filter.max_particles_loc << std::endl;

                    if (it != node->coordinate_map.end()) {
                        double x = std::get<0>(it->second);
                        double y = std::get<1>(it->second);
                        t.transform.translation.x = x;
                        t.transform.translation.y = y;
                        t.transform.translation.z = std::get<2>(it->second);

                        particle_filter.patient_x = x;
                        particle_filter.patient_y = y;

                    } else {
                        particle_filter.patient_x = std::nan("");
                        particle_filter.patient_y = std::nan("");
                        // Handle the case where the landmark is not found in the map
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Landmark %s not found in the map!",
                                     particle_filter.max_particles_loc.c_str());
                    }

                    t.transform.rotation.x = 0;
                    t.transform.rotation.y = 0;
                    t.transform.rotation.z = 0;
                    t.transform.rotation.w = 1;
                    t.header.frame_id = "map";


                } else {

                    std::vector<Observation> observations;
                    observations.push_back(obs_);

                    std::string cam_name = obs_.name;
                    std::cout << "  cam_name  " << cam_name << std::endl;
                    auto extrinsicParams = camera_extrinsics[cam_name];

                    // Update the weights and resample
                    particle_filter.updateWeightsWithObs(sigma_landmark, observations, extrinsicParams );
                    std::cout << " updateWeightsWithObs  " << std::endl;

//                    double Neff = particle_filter.calculateNeff();
//                    // resample if too few effective particles
//                    std::cout << " Neff:  " << Neff << std::endl;
//                    std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

//                    if (Neff < (particle_filter.num_particles) / 3) {
                    std::cout << " resample  " << std::endl;

//                    node->publish_particles(particle_filter.particles);
                    particle_filter.resample();
                    particle_filter.check_unique_particles();
                    node->publish_particles(particle_filter.particles);

//                    }

                    // publish particle with highest weight
                    double highest_weight = 0.0;

                    Particle best_particle;

                    for (int i = 0; i < particle_filter.particles.size(); ++i) {
                        if (particle_filter.particles[i].weight > highest_weight) {
                            highest_weight = particle_filter.particles[i].weight;
                            best_particle = particle_filter.particles[i];
                            particle_filter.patient_x = best_particle.x;
                            particle_filter.patient_y = best_particle.y;

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

                }

                std::cout << "t.transform.translation.x: " << t.transform.translation.x << std::endl;
                std::cout << "t.transform.translation.y: " << t.transform.translation.y << std::endl;


//                node->publish_particles(particle_filter.particles);
                t.header.stamp = rclcpp::Clock().now();
                tf_broadcaster_->sendTransform(t);
                std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%% " << std::endl;
                std::cout << "  TRANSFORM &*777 " << std::endl;
                //observation camera
            }

        }

        rclcpp::spin_some(node);
    }

//    thread_1.join();
    rclcpp::shutdown();

    return 0;
}

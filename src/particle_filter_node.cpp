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
    double sigma_landmark[3] = {0.04, 0.04, 0.04};

//    auto world_state_converter = std::make_shared<WorldStateListener>("WorldStatePDDLConverter", param_listener_);
//    std::thread thread_1(
//            [&world_state_converter]() {
//                rclcpp::executors::MultiThreadedExecutor executor;
//                executor.add_node(world_state_converter);
//                while (!world_state_converter->should_terminate_node()) {
//                    executor.spin_some();
//                }
//            }
//    );

    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
    node->publish_particles(particle_filter.particles);

    while (rclcpp::ok()) {
        if (not_initialized) {
            camera_extrinsics = node->get_cam_extrinsic_matrix();
            // make sure camera positions are set
            if (camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {

            door_status_ = node->getdoorstatus();
            // Initialize the particle filter in a uniform distribution


            obs_ = node->getObservation();
            first_obs = node->first_obs;

            // for debug
            first_obs = true;
            // NO FIRST OBSERVATION KEEP DISTRIBUTION AS IS
            if (first_obs) {

                // Fill in the message
                geometry_msgs::msg::TransformStamped t;
                /// should be whatever the code is expecting the name to be
                t.child_frame_id = "nathan";

                particle_filter.motion_model_noisy(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_);

                if (obs_.name.empty()) {
                    //observation empty

                    // Update the weights and resample
                    particle_filter.updateWeightsWithoutObs(sigma_landmark);
                    std::cout << " updateWeightsWithoutObs  " << std::endl;

                    double Neff = particle_filter.calculateNeff();
                    // resample if too few effective particles
                    std::cout << " Neff:  " << Neff << std::endl;
                    std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

                    if (Neff < (particle_filter.num_particles) / 3) {
                        std::cout << " resample  " << std::endl;

                        particle_filter.resample();
                    }

//                  publish location  in the location with the most particles
                    auto it = node->coordinate_map.find(particle_filter.max_particles_loc);
                    std::cout << "max_loc _ " << particle_filter.max_particles_loc << std::endl;

                    if (it != node->coordinate_map.end()) {
                        t.transform.translation.x = std::get<0>(it->second);
                        t.transform.translation.y = std::get<1>(it->second);
                        t.transform.translation.z = std::get<2>(it->second);
                    } else {
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
                    std::vector<Particle> particles = particle_filter.particles;

                    std::vector<Observation> observations;
                    observations.push_back(obs_);

                    std::string cam_name = obs_.name;
                    std::cout << "  cam_name  " << cam_name << std::endl;
                    auto extrinsicParams = camera_extrinsics[cam_name];

                    // Update the weights and resample
                    particle_filter.updateWeightsWithObs(sigma_landmark, observations, extrinsicParams );
                    std::cout << " updateWeightsWithoutObs  " << std::endl;

                    double Neff = particle_filter.calculateNeff();
                    // resample if too few effective particles
                    std::cout << " Neff:  " << Neff << std::endl;
                    std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

                    if (Neff < (particle_filter.num_particles) / 3) {
                        std::cout << " resample  " << std::endl;

                        particle_filter.resample();
                    }

                    // publish particle with highest weight
                    double highest_weight = 0.0;

                    Particle best_particle;

                    for (int i = 0; i < particles.size(); ++i) {
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

                }

                node->publish_particles(particle_filter.particles);
                t.header.stamp = rclcpp::Clock().now();
                tf_broadcaster_->sendTransform(t);
                //observation camera
            }


        }

        rclcpp::spin_some(node);
    }

//    thread_1.join();
    rclcpp::shutdown();

    return 0;
}

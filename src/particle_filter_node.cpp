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

    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
    node->publish_particles(particle_filter.particles);
//    std::vector<Particle> particles = particle_filter.particles;

    while (rclcpp::ok()) {
        if (not_initialized) {
            camera_extrinsics = node->get_cam_extrinsic_matrix();
            // make sure camera positions are set
            if (camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {

            door_status_ = node->getdoorstatus();
            ms_status_ = node->getmsstatus();

            obs_ = node->getObservation(particle_filter);
            first_obs = node->first_obs;

            // for debug
//            first_obs = true;
            // NO FIRST OBSERVATION KEEP DISTRIBUTION AS IS
            if (first_obs) {

                // Fill in the message
                std_msgs::msg::Float64MultiArray message;
//                std::cout << " obs_.name *******  " << obs_.name << std::endl;

//                node->publish_particles(particle_filter.particles);

                if (obs_.name.empty() ) {
                    particle_filter.motion_model_noisy(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_, obs_.name);
                    particle_filter.apply_special_transitions(door_status_, node->currentStateH, ms_status_);

                    // Update the weights and resample
                    particle_filter.updateWeightsWithoutObs(sigma_landmark);
                    node->publish_particles(particle_filter.particles);

//                        std::cout << " updateWeightsWithoutObs  " << std::endl;

                    double Neff = particle_filter.calculateNeff();
                    // resample if too few effective particles
//                        std::cout << " Neff:  " << Neff << std::endl;
//                        std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

                    if (Neff < particle_filter.num_particles / 3) {
//                            std::cout << " resample due to Neff dropping below 1/3  " << std::endl;
                        particle_filter.resample();
                        particle_filter.check_unique_particles();
                        node->publish_particles(particle_filter.particles);

                    }

                    //  publish location  in the location with the most particles
                    auto it = node->coordinate_map.find(particle_filter.max_particles_loc);
                    //  std::cout << "max_loc _ " << particle_filter.max_particles_loc << std::endl;

                    if (it != node->coordinate_map.end()) {
                        double x = std::get<0>(it->second);
                        double y = std::get<1>(it->second);
                        // double z = std::get<2>(it->second);

                        message.data = {x, y};

                        particle_filter.patient_x = x;
                        particle_filter.patient_y = y;

                    } else {
                        particle_filter.patient_x = std::nan("");
                        particle_filter.patient_y = std::nan("");
                        // Handle the case where the landmark is not found in the map
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Landmark %s not found in the map!",
                                     particle_filter.max_particles_loc.c_str());
                    }



                } else {
//                    std::cout << " ((((( observatiopn  " << std::endl;
                    // only observations undo transportations
                    particle_filter.motion_model_noisy(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_, obs_.name);
                    particle_filter.special_transitions_monitoring(door_status_, ms_status_, node->person_at_doorway);



                    std::vector<Observation> observations;
                    observations.push_back(obs_);

                    std::string cam_name = obs_.name;
//                    std::cout << "  cam_name  " << cam_name << std::endl;
                    auto extrinsicParams = camera_extrinsics[cam_name];

                    // Update the weights and resample
                    particle_filter.updateWeightsWithObs(sigma_landmark, observations, extrinsicParams );
//                    std::cout << " updateWeightsWithObs  " << std::endl;

//                    double Neff = particle_filter.calculateNeff();
//                    // resample if too few effective particles
//                    std::cout << " Neff:  " << Neff << std::endl;
//                    std::cout << " N/3:  " << (particle_filter.num_particles) / 3 << std::endl;

//                    if (Neff < (particle_filter.num_particles) / 3) {
//                    std::cout << " resample  " << std::endl;

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

                    message.data = {best_particle.x, best_particle.y};
//                    std::cout << "from observation publish_person_loc" << std::endl;

                }

                // std::cout << "t.transform.translation.x: " << t.transform.translation.x << std::endl;
                // std::cout << "t.transform.translation.y: " << t.transform.translation.y << std::endl;


//                node->publish_particles(particle_filter.particles);
                node->publish_person_loc->publish(message);
//                std::cout << "publish_person_loc" << std::endl;

                //observation camera
            }

        }

        rclcpp::spin_some(node);
    }

//    thread_1.join();
    rclcpp::shutdown();

    return 0;
}

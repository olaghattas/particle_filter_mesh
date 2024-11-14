//
// Created by ola on 6/14/23.
//
#include <random>
#include <algorithm>
#include <map>
#include <numeric>
#include "particle_filter/particle_filter.h"
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
// needed for the projection
#include <iostream>
#include "shr_utils/geometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define EPSILON 1e-4

void ParticleFilter::normalize_weights() {
    double sum_weights = 0.0;

    // Compute the sum of all particle weights
    for (const Particle &particle: particles) {
        sum_weights += particle.weight;
    }

    // Normalize the weights so that they sum up to one
    for (Particle &particle: particles) {
        if (sum_weights > 0) {
            particle.weight /= sum_weights;
        } else {
            particle.weight = 1.0 / particles.size(); // Assign equal weight if sum is zero
        }
    }
}

// Function to find the landmark with the most particles
std::string ParticleFilter::find_landmark_with_most_particles() {
    std::vector<std::string>
            lndmarks = {"living_room", "bedroom", "outside"};

    std::map<std::string, int> particle_count;

    // Initialize the count for each landmark
    for (const auto &landmark: lndmarks) {
        particle_count[landmark] = 0;
    }

    // Count particles in each landmark
    for (const auto &particle: particles) {
        Eigen::Vector3d point = {particle.x, particle.y, -0.5};
        for (const auto &landmark: lndmarks) {
            if (check_particle_room(landmark, point)) {
                particle_count[landmark]++;
                break;
            }
        }
    }
    // Find the landmark with the highest number of particles
    auto max_landmark_it = std::max_element(particle_count.begin(), particle_count.end(),
                                            [](const std::pair<std::string, int> &a,
                                               const std::pair<std::string, int> &b) {
                                                return a.second < b.second;
                                            });

    if (max_landmark_it != particle_count.end()) {
        return max_landmark_it->first;
    } else {
        // Handle the case where no landmarks are found
        std::cout << "NO LANDMARK" << std::endl;
        return "";
    }
}


void ParticleFilter::write_to_file(std::string filename) {
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {

        // Write data to the file with multiple lines and variables
        for (int i = 0; i < num_particles; i++) {
            outputFile << "id: " << particles[i].id << "  x: " << particles[i].x << "  y: " << particles[i].y
                       << "  weight: " << particles[i].weight << std::endl;
        }

        outputFile.close();
        std::cout << "Data has been written to the file." << std::endl;
    } else {
        std::cerr << "Error opening the file." << std::endl;
    }

}

void ParticleFilter::init(std::pair<double, double> x_bound, std::pair<double, double> y_bound,
                          std::pair<double, double> z_bound,
                          std::pair<double, double> theta_bound) {
    avg_displacement(0.0, 0.0);
    previous_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    current_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    previous_count = 0;
    min_weight = 0.000001;

    // Add random Gaussian noise to each particle.
    //    std::default_random_engine gen;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::uniform_real_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    particles.clear();
    weights.clear();
    for (int i = 0; i < num_particles; ++i) {
        Particle p = {i, xNoise(gen), yNoise(gen), zNoise(gen), yawNoise(gen), 1.0};
        particles.push_back(p);
        weights.push_back(1);

    }
//    write_to_file("first_run.txt");
    is_initialized = true;

    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter_mesh");

    auto mesh_file = (pkg_dir / "config" / "olson_collision_mesh.obj").string();

    auto [mesh_verts, mesh_names] = shr_utils::load_meshes(mesh_file);
    for (int i = 0; i < mesh_names.size(); i++) {
        auto name = mesh_names[i];
        auto verts = mesh_verts[i];
        mesh_vert_map_[name] = verts;
    }

    auto view_points_mesh_file = (pkg_dir / "config" / "olson_cam_sep.obj").string();

    auto [view_points_mesh_verts, view_points_mesh_names] = shr_utils::load_meshes(view_points_mesh_file);
    for (int i = 0; i < view_points_mesh_names.size(); i++) {
        auto name_mesh = view_points_mesh_names[i];
        auto verts_mesh = view_points_mesh_verts[i];
        view_points_mesh_vert_map_[name_mesh] = verts_mesh;
    }

    auto room_mesh_file = (pkg_dir / "config" / "new_olson_person.obj").string();

    auto [room_mesh_verts, room_mesh_names] = shr_utils::load_meshes(room_mesh_file);
    for (int i = 0; i < room_mesh_names.size(); i++) {
        auto name_room = room_mesh_names[i];
        auto verts_room = room_mesh_verts[i];
        mesh_vert_map_room[name_room] = verts_room;
    }

}

void ParticleFilter::particles_in_range(std::pair<double, double> x_bound, std::pair<double, double> y_bound,
                                        int ind_start) {
    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);

    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = ind_start; i < ind_start + 10; ++i) {
        particles[i].x = xNoise(gen);
        particles[i].y = yNoise(gen);

    }
}

//void ParticleFilter::motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
//                                  std::vector<bool> doors_status, std::string observation) {
//    std::default_random_engine gen;
////    std::normal_distribution<double> xNoise(0, std_pos[0]);
////    std::normal_distribution<double> yNoise(0, std_pos[1]);
////    std::normal_distribution<double> zNoise(0, std_pos[2]);
////    std::normal_distribution<double> yawNoise(0, std_pos[3]);
//
//    std::normal_distribution<double> xNoise(0, 0.25);
//    std::normal_distribution<double> yNoise(0, 0.25);
//    std::normal_distribution<double> zNoise(0, 0.03);
//    std::normal_distribution<double> yawNoise(0, 0.03);
//
//
//    auto particles_before = particles;
//    for (auto &p: particles) {
//        // only 80 percent of the particle will be directed in the direction of the vector the rest will be random
//        // Calculate average displacement vector from previous readings
//
//        ///
//        // if current and previous have NAN then randomly distribute
//        // if current has value and previous hasNAN then randomly distribute
//        // (1) ==>  can be summarized to previous hasNAN then randomly distribute
//
//        // (2) if current and previous have values then displace in the direction of vector
//        // (3) if current isNaN and previous has value then update according to displacement for 5 iteration then set previous to NAN
//        ///
//        if (!previous_observation.hasNaN()) {  // (1)
//            // && p.id < num_particles * 0.8) {
//            use_max_loc = false;
//            if (!current_observation.hasNaN()) { // (2)
//                // Extract x and y coordinates from each reading
//                double dx = current_observation.x() - previous_observation.x();
//                double dy = current_observation.y() - previous_observation.y();
//                avg_displacement = Eigen::Vector2d(dx, dy);
//
//                // Normalize average displacement for velocity calculation
//                // double avg_disp = avg_displacement.norm();
//
//                // Update particle position and orientation using avg_direction and velocity
//                double delta_x = avg_displacement.x() + xNoise(gen);
//                double delta_y = avg_displacement.y() + yNoise(gen);
//                double delta_yaw = yawNoise(gen);
//                p.x += delta_x;
//                p.y += delta_y;
//                p.theta += delta_yaw;
//            } else { // (3)
//                // Use previous displacement
//                double delta_x = avg_displacement.x() + xNoise(gen);
//                double delta_y = avg_displacement.y() + yNoise(gen);
//                double delta_yaw = yawNoise(gen);
//                p.x += delta_x;
//                p.y += delta_y;
//                p.theta += delta_yaw;
//
//                if (previous_count < 5) {
//                    previous_count++;
//                } else {
//                    previous_count = 0;
//                    previous_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
//                }
//            }
//
//        } else {
//            // add noise randomly
//            //Add control noise
//            double delta_x = xNoise(gen); //* delta_t;
//            double delta_y = yNoise(gen); // * delta_t;
////            double delta_z = zNoise(gen); // * delta_t;
//            double delta_yaw = yawNoise(gen); // * delta_t;
//
//            p.x += delta_x;
//            p.y += delta_y;
//            p.z += 0;
//            p.theta += delta_yaw;
//
//
//            /// NO current observation
//
//            if (!use_max_loc){
//                // if it was not already calculated then check which room has the highest number of particles
//                // no need to recalculate cause this value won't change unless an observation is made which will cause
//                // the upper part  of the if to change use_max_loc to false
//                max_particles_loc = find_landmark_with_most_particles();
//                std::cout << "max_loc _ " << max_particles_loc << std::endl;
//                use_max_loc = true;
//            }
//        }
//    }
//
//
//    ParticleFilter::enforce_non_collision(particles_before, doors_status, observation
//    );
//
////    write_to_file("after_motion_model.txt");
//}


void ParticleFilter::motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                                  std::vector<bool> doors_status, std::string observation) {
    std::default_random_engine gen;

    std::normal_distribution<double> xNoise(0, 0.03);
    std::normal_distribution<double> yNoise(0, 0.03);
    std::normal_distribution<double> zNoise(0, 0.03);
    std::normal_distribution<double> yawNoise(0, 0.03);


    auto particles_before = particles;
//    std::cout << " x _before" << particles[0].x << " y_before" << particles[0].y << std::endl;
    //write_to_file("before_motion_model.txt");
    for (auto &p: particles) {

        //Add control noise
        double delta_x = xNoise(gen); //* delta_t;
        double delta_y = yNoise(gen);// * delta_t;
        double delta_z = zNoise(gen);// * delta_t;
        double delta_yaw = yawNoise(gen);// * delta_t;


        p.x += delta_x;
        p.y += delta_y;
        p.z += 0;
        p.theta += delta_yaw;
    }

//    std::cout << " x _after" << particles[0].x << " y_after" << particles[0].y << std::endl;

    // to be passed in through arguments
    ParticleFilter::enforce_non_collision(particles_before, doors_status);

}


float ParticleFilter::sample(float mean, float variance) {
    // randomly sample from a Normal distribution
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<> dist(mean, sqrt(variance));
    return dist(gen);
}

void ParticleFilter::resample() {
    // low variance resampler
//    write_to_file("before_resampling.txt");

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles);

    std::vector<Particle> resampled_particles = particles;

    double c = particles[0].weight;
    int i = 0;
    float r = double(dist(gen));

    for (int m = 0; m < num_particles; m++) {
        float u = r + (float) m / num_particles;
        while (u > c && i < num_particles) {
            i++;
            c += particles[i].weight;
        }
        resampled_particles[m].weight = particles[i].weight;
        resampled_particles[m].x = particles[i].x;
        resampled_particles[m].y = particles[i].y;

//        resampled_particles[m].weight = 1.0 / num_particles;
    }
    particles = resampled_particles;
    normalize_weights();
//    write_to_file("after_resampling.txt");
}

void ParticleFilter::updateWeights(double std_landmark[],
                                   std::vector<Observation> observations) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_z = std_landmark[2];
    double weights_sum = 0;

    if (!current_observation.hasNaN()) {
        previous_observation = current_observation;
    }

    // observation camera:
    if (!current_observation.hasNaN()) {
        Observation current_obs = observations[0]; // TODO be changed when more observations are added
        Eigen::Vector4d homogeneousPoint;
        homogeneousPoint << current_obs.x, current_obs.y, current_obs.z, 1.0;

//        Eigen::Vector4d TransformedPoint;

//        std::cout << "update2 &&&&&&&&&&&&&&&&&&&&&& " << std::endl;
//        TransformedPoint <<
//                         extrinsicParams(0, 0) * homogeneousPoint[0] + extrinsicParams(0, 1) * homogeneousPoint[1] +
//                         extrinsicParams(0, 2) * homogeneousPoint[2] + extrinsicParams(0, 3) * homogeneousPoint[3],
//                extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
//                extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
//                extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
//                extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
//                extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
//                extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];


        /// ONLY ONE OBSERVATION AT A TIME
//        current_observation = Eigen::Vector2d(TransformedPoint[0], TransformedPoint[1]);
        current_observation = Eigen::Vector2d(current_obs.x, current_obs.y);

        // loop through each of the particle to update
        for (int i = 0; i < num_particles; ++i) {
            Particle *p = &particles[i];
            double weight = 1.0;

//            double gaussian = (((p->x - TransformedPoint[0]) * (p->x - TransformedPoint[0])) /
//                               (2 * sigma_x * sigma_x)) +
//                              (((p->y - TransformedPoint[1]) * (p->y - TransformedPoint[1])) /
//                               (2 * sigma_y * sigma_y));

            double x_ = p->x - current_obs.x;
            double y_ = p->y - current_obs.y;
//            double gaussian = (((p->x - current_obs.x) * (p->x - current_obs.x)) /
//                               (2 * sigma_x * sigma_x)) +
//                              (((p->y - current_obs.y) * (p->y - current_obs.y)) /
//                               (2 * sigma_y * sigma_y));
//            if(abs(x_)>0.1){
//                sigma_x = 0.4;
//            }else{
//                sigma_x = 0.04;
//            }
//            if(abs(y_)>0.1){
//                sigma_y = 0.4;
//            }else{
//                sigma_y = 0.04;
//            }
            // below gives the order of mag of the diff but i might want to multiply by number
            // factor is for that
            // weird behaviour happens when sigma has mag > x because the exp =0 and weigh = 0
            double factor = 4;

            // Dynamically compute sigma based on the order of magnitude of x_ and y_
            double sigma_x = std::pow(10, std::floor(std::log10(std::abs(x_))) - 1); // Order of magnitude for x_
            double sigma_y = std::pow(10, std::floor(std::log10(std::abs(y_))) - 1);

            double gaussian = (std::pow(x_, 2) / (2 * factor * std::pow(sigma_x, 2))) +
                              (std::pow(y_, 2) / (2 * std::pow(sigma_y, 2)));
            double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y);
            gaussian = exp(-gaussian);
            gaussian = gaussian * gaussian_factor;

            weight *= gaussian;
            weights_sum += weight;
            particles[i].weight = weight;

        }
    } else {
        std::vector<std::string> view_point_ = {"corr_cam", "door_cam", "kitchen_cam", "living_cam"};
        double factor = 0.1;
        for (int i = 0; i < num_particles; ++i) {
            bool in_camview = false;
            Eigen::Vector3d point = {particles[i].x, particles[i].y, 0.0};
            // person shouldnt be in camera

            for (const std::string &view: view_point_) {
                if (check_particle_at_cam_view(view, point)) {
//                    std::cout << "particles[i].weight" << particles[i].weight << std::endl;
                    particles[i].weight = std::max(particles[i].weight - particles[i].weight * factor, min_weight);
//                    std::cout << "particles[i].weight" << particles[i].weight << std::endl;
                    in_camview = true;
                }
            }
            if (!in_camview) {
                particles[i].weight = particles[i].weight * factor;
            }
            weights_sum += particles[i].weight;
        }
    }
//    std::cout << "{door_bedroom, door_bathroom, door_outdoor}" << doors_status[0] << " " << doors_status[1] << " "
//              << doors_status[2] << std::endl;

//        for (int i = 0; i < num_particles; ++i) {
//            Eigen::Vector3d point = {particles[i].x, particles[i].y, -0.5};
//                observation == "" && check_particle_at_cam_view(view_point_[0], point)) {
//                particles[i] = old_particles[i];
//                particles[i].weight = 0.0;
//
//            }

    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        particles[i].weight /= weights_sum;
    }


}


bool ParticleFilter::check_particle_at(const std::string &loc, Eigen::Vector3d point) {
    if (mesh_vert_map_.find(loc) == mesh_vert_map_.end()) {
        return false;
    }
    auto verts = mesh_vert_map_.at(loc);
    Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
    return shr_utils::PointInMesh(point, verts, verts2d);
}

bool ParticleFilter::check_particle_room(const std::string &loc, Eigen::Vector3d point) {
    if (mesh_vert_map_room.find(loc) == mesh_vert_map_room.end()) {
        return false;
    }
    auto verts = mesh_vert_map_room.at(loc);
    Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
    return shr_utils::PointInMesh(point, verts, verts2d);
}


bool ParticleFilter::check_particle_at_cam_view(const std::string &loc, Eigen::Vector3d point) {
    if (view_points_mesh_vert_map_.find(loc) == view_points_mesh_vert_map_.end()) {
        return false;
    }
    auto verts = view_points_mesh_vert_map_.at(loc);
    Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
    return shr_utils::PointInMesh(point, verts, verts2d);
}

void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles,
                                           std::vector<bool> doors_status) {

    std::vector<std::string>
            lndmarks = {"obstacles", "bedroom", "bathroom", "main_door"};
//    std::vector<std::string> view_point = {"visible_area"};

    for (int i = 0; i < num_particles; ++i) {
        Eigen::Vector3d point = {particles[i].x, particles[i].y, -0.5};

        // ###### COLLSIONS WITH OBSTACLES ########
        if (check_particle_at(lndmarks[0], point)) {
            // obstacle (not door)
            particles[i] = old_particles[i];
            particles[i].weight = 0.0;

        } else if (check_particle_at(lndmarks[1], point)) {
            // bedroom_door
            // the index should correspond to the door in door status found in article_filter_node.cpp
            std::cout << "door bedroom_door status " << doors_status[0] << std::endl;
            // door_status = 1 if door is open
            // when closed particles should not go in
            if (!doors_status[0]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[2], point)) {
            // bathroom
            // the index should correspond to the door in door status found in article_filter_node.cpp
            std::cout << "door bathroom status " << doors_status[1] << std::endl;
            if (!doors_status[1]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[3], point)) {
            // main door
            // the index should correspond to the door in door status found in article_filter_node.cpp
            std::cout << "door outdoor status " << doors_status[2] << std::endl;
            if (!doors_status[2]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        }

        // handled iin weight updates since no one in camera still acts as an observation
// ###### POINTS GOING INTO CAMERA VIEW POINT WHEN NO PERSON IS THERE ########
// doesnt allow the particle to go into view points when no observation in camera
        /// IF Observation empty then particles cant be in the viewed area
//        else if (observation == "" && check_particle_at_cam_view(view_point[0], point)) {
//            particles[i] = old_particles[i];
//            particles[i].weight = 0.0;
//        }
    }
}
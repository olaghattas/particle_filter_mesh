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

    auto mesh_file = (pkg_dir / "config" / "sajay_coll_new.obj").string();
//    std::string mesh_file = "/home/olagh/smart-home/src/smart-home/external/particle_filter_mesh/config/collision_mesh.obj";

    auto [mesh_verts, mesh_names] = shr_utils::load_meshes(mesh_file);
    for (int i = 0; i < mesh_names.size(); i++) {
        auto name = mesh_names[i];
        auto verts = mesh_verts[i];
        mesh_vert_map_[name] = verts;
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

void ParticleFilter::motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                                  std::vector<bool> doors_status) {
    std::default_random_engine gen;
//    std::normal_distribution<double> xNoise(0, std_pos[0]);
//    std::normal_distribution<double> yNoise(0, std_pos[1]);
//    std::normal_distribution<double> zNoise(0, std_pos[2]);
//    std::normal_distribution<double> yawNoise(0, std_pos[3]);

    std::normal_distribution<double> xNoise(0, 0.01);
    std::normal_distribution<double> yNoise(0, 0.01);
    std::normal_distribution<double> zNoise(0, 0.03);
    std::normal_distribution<double> yawNoise(0, 0.03);


    auto particles_before = particles;
    // std::cout << " x _before" << particles[0].x << " y_before" << particles[0].y << std::endl;
//    write_to_file("before_motion_model.txt");
    for (auto &p: particles) {
        // only 80 percent of the oarticle will be directed in the direction of the vector the rest will be random
        // Calculate average displacement vector from available readings
        if (previous_observation.size() > 2) { //&& p.id < num_particles * 0.8) {
            Eigen::Vector2d avg_displacement(0.0, 0.0);
            for (int i = 0; i < previous_observation.size(); i++) {
                // Extract x and y coordinates from each reading
                double dx = previous_observation[i].x() - p.x;
                double dy = previous_observation[i].y() - p.y;
                avg_displacement += Eigen::Vector2d(dx, dy);
            }
            avg_displacement /= previous_observation.size();

            // Normalize average displacement for velocity calculation

//            double avg_distance = avg_displacement.norm();

            // Update particle position and orientation using avg_direction and velocity
            double delta_x = avg_displacement.x() + xNoise(gen);
            double delta_y = avg_displacement.y() + yNoise(gen);
            double delta_yaw = yawNoise(gen);
            p.x += delta_x;
            p.y += delta_y;
            p.theta += delta_yaw;

        } else {
            // add noise randomly
            //Add control noise
            double delta_x = xNoise(gen); //* delta_t;
            double delta_y = yNoise(gen); // * delta_t;
//            double delta_z = zNoise(gen); // * delta_t;
            double delta_yaw = yawNoise(gen); // * delta_t;

            p.x += delta_x;
            p.y += delta_y;
            p.z += 0;
            p.theta += delta_yaw;
        }
    }


    // std::cout << " x _after" << particles[0].x << " y_after" << particles[0].y << std::endl;

    // to be passed in through arguments

    // use last 20 particle in areas where cameras can view
    // for now this will happen at all times later will dedpend on observation availability
    if (no_readings) {
        std::pair<double, double> x_kitchen_bound = std::make_pair(0, 2.0);
        std::pair<double, double> y_kitchen_bound = std::make_pair(-2, 0.25);
        std::pair<double, double> x_dining_bound = std::make_pair(0.6, 2.3);
        std::pair<double, double> y_dining_bound = std::make_pair(1.5, 3.7);

        ParticleFilter::particles_in_range(x_kitchen_bound, y_kitchen_bound, 0);
        ParticleFilter::particles_in_range(x_dining_bound, y_dining_bound, 10);
    }

    ParticleFilter::enforce_non_collision(particles_before, doors_status);

//    write_to_file("after_motion_model.txt");

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

        resampled_particles[m].weight = 1.0 / num_particles;
    }
    particles = resampled_particles;
//    write_to_file("after_resampling.txt");

//    for (int m = 0; m < num_particles; m++) {
//        double max_weight = 0.00;
//        int count = 0;
//            resampled_particles[i]
//        }
}

void ParticleFilter::updateWeights(double std_landmark[],
                                   std::vector<Observation> observations,
                                   Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_z = std_landmark[2];
    double weights_sum = 0;
//    write_to_file("before_weight_update.txt");


    Observation current_obs = observations[0]; // TODO be changed when more observations are added
    Eigen::Vector4d homogeneousPoint;
    homogeneousPoint << current_obs.x, current_obs.y, current_obs.z, 1.0;

    Eigen::Vector4d TransformedPoint;

    std::cout << "update2 &&&&&&&&&&&&&&&&&&&&&& " << std::endl;
    TransformedPoint <<
                     extrinsicParams(0, 0) * homogeneousPoint[0] + extrinsicParams(0, 1) * homogeneousPoint[1] +
                     extrinsicParams(0, 2) * homogeneousPoint[2] + extrinsicParams(0, 3) * homogeneousPoint[3],
            extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
            extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
            extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
            extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
            extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
            extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];

    // std::cout << " Observation ::: x " << TransformedPoint[0] << " y " << TransformedPoint[1] << " z "
    //           << TransformedPoint[2] << std::endl;

    if (previous_observation.size() < 10)
        previous_observation.push_back(Eigen::Vector2d(TransformedPoint[0], TransformedPoint[1]));
    else {
        // Remove the oldest observation
        previous_observation.erase(previous_observation.begin());

        // Add the newest observation
        previous_observation.push_back(Eigen::Vector2d(TransformedPoint[0], TransformedPoint[1]));
    }

    // loop through each of the particle to update
    for (int i = 0; i < num_particles; ++i) {
        Particle *p = &particles[i];
        double weight = 1.0;

        // update weights using Multivariate Gaussian Distribution
        // equation given in Transformations and Associations Quiz
//        double gaussian = ((p->x - TransformedPoint[0]) * (p->x - TransformedPoint[0]) /
//                           (2 * sigma_x * sigma_x)) +
//                          ((p->y - TransformedPoint[1]) * (p->y - TransformedPoint[1]) /
//                           (2 * sigma_y * sigma_y)) + ((p->z - TransformedPoint[2]) * (p->z - TransformedPoint[2]) /
//                                                       (2 * sigma_z * sigma_z));
//        double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y * sigma_z);
        double gaussian = (((p->x - TransformedPoint[0]) * (p->x - TransformedPoint[0])) /
                           (2 * sigma_x * sigma_x)) +
                          (((p->y - TransformedPoint[1]) * (p->y - TransformedPoint[1])) /
                           (2 * sigma_y * sigma_y));

        double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y);
        gaussian = exp(-gaussian);
        gaussian = gaussian * gaussian_factor;

        weight *= gaussian;
        weights_sum += weight;
        particles[i].weight = weight;
    }

    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        particles[i].weight /= weights_sum;
    }
    //write_to_file("after_weight_update.txt");

    // Update observations
}


bool ParticleFilter::check_particle_at(const std::string &loc, Eigen::Vector3d point) {
    if (mesh_vert_map_.find(loc) == mesh_vert_map_.end()) {
        return false;
    }
    auto verts = mesh_vert_map_.at(loc);
    Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
    return shr_utils::PointInMesh(point, verts, verts2d);
}

void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles,
                                           std::vector<bool> doors_status) {

    // LANDMARK ORDER SHOULD MATCH DOOR STATUS ORDER
    std::vector<std::string>
            lndmarks = {"obstacles", "bedroom_door", "bathroom_door", "door", "obstacle_1", "obstacle_3"};
//    std::cout << "{door_bedroom, door_bathroom, door_outdoor}" << doors_status[0] << " " << doors_status[1] << " "
//              << doors_status[2] << std::endl;

    for (int i = 0; i < num_particles; ++i) {
        Eigen::Vector3d point = {particles[i].x, particles[i].y, -0.5};
        if (check_particle_at(lndmarks[0], point)) {
            // obstacle (not door)
            particles[i] = old_particles[i];
            particles[i].weight = 0.0;

        } else if (check_particle_at(lndmarks[1], point)) {
            // bedroom_door
            if (doors_status[0]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[2], point)) {
            // bathroom_door
            if (doors_status[1]) {
                // door 1 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[3], point)) {
            // outside_door
            if (doors_status[2]) {
                // door 1 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[4], point)) {
            // obstacle  1 (not door)
            particles[i] = old_particles[i];
            particles[i].weight = 0.0;

        } else if (check_particle_at(lndmarks[5], point)) {
            // obstacle  3 (not door)
            particles[i] = old_particles[i];
            particles[i].weight = 0.0;

        }
    }
}


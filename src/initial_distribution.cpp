//
// Created by ola on 6/15/23.
//

#include <functional>
#include <memory>
#include <string>
#include "particle_filter.cpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include <Eigen/Dense>

#include <random>
#include <array>
#include <particle_filter/particle_filter_node.hpp>
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();

    std::pair<double, double> x_bound = std::make_pair(-1, 1);
    std::pair<double, double> y_bound = std::make_pair(-1, 1);

    std::pair<double, double> z_bound = std::make_pair(0, 0);
    std::pair<double, double> theta_bound = std::make_pair(-3.1416, 3.1416);

    int num_particles = 500;
    ParticleFilter particle_filter(num_particles);

    // init randomly
    std::random_device rd;
    std::mt19937 gen;

    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::uniform_real_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    for (int i = 0; i < particle_filter.num_particles; ++i) {
        Particle p = {i, xNoise(gen), yNoise(gen), zNoise(gen), yawNoise(gen), 1.0};
        particle_filter.particles.push_back(p);
        particle_filter.weights.push_back(1);
    }

    node->publish_particles(particle_filter.particles);
//    std::this_thread::sleep_for(std::chrono::seconds(10));
//    std::vector<Particle> particles = particle_filter.particles;
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter_mesh");

    auto mesh_file = (pkg_dir / "config" / "gaskin_collision_mesh.obj").string();

    auto [mesh_verts, mesh_names] = shr_utils::load_meshes(mesh_file);
    for (int i = 0; i < mesh_names.size(); i++) {
        auto name = mesh_names[i];
        auto verts = mesh_verts[i];
        particle_filter.mesh_vert_map_[name] = verts;
    }
    std::vector<std::string>
            lndmarks = {"obstacles", "main_door", "bedroom_door", "trash_door","backdoor"};

    while (rclcpp::ok()) {
    //        motion_model_noisy
            auto particles_before = particle_filter.particles;
            for (auto &p: particle_filter.particles) {

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

            for (int i = 0; i < particle_filter.num_particles; ++i) {
                Eigen::Vector3d point = {particle_filter.particles[i].x, particle_filter.particles[i].y, -0.5};

                // ###### COLLISIONS WITH OBSTACLES ########
                if (particle_filter.check_particle_at(lndmarks[0], point)) {
                    // obstacle (not door)
                    particle_filter.particles[i] = particles_before[i];
                    particle_filter.particles[i].weight = 0.0;
                } else if (particle_filter.check_particle_at(lndmarks[1], point)) {
                    particle_filter.particles[i] = particles_before[i];
                    particle_filter.particles[i].weight = 0.0;
                } else if (particle_filter.check_particle_at(lndmarks[2], point)) {
                    particle_filter.particles[i] = particles_before[i];
                    particle_filter.particles[i].weight = 0.0;
                } else if (particle_filter.check_particle_at(lndmarks[3], point)) {
                    particle_filter.particles[i] = particles_before[i];
                    particle_filter.particles[i].weight = 0.0;
                }else if (particle_filter.check_particle_at(lndmarks[4], point)) {
                    particle_filter.particles[i] = particles_before[i];
                    particle_filter.particles[i].weight = 0.0;
                }

            }
            node->publish_particles(particle_filter.particles);
            std::string initial_dist_ = "/home/olagh48652/particle_filter_ws/src/particle_filter_mesh/config/initial_dist.txt";
            std::ofstream outputFile(initial_dist_);

            if (outputFile.is_open()) {
                // Write data to the file with multiple lines and variables
                for (int i = 0; i < particle_filter.num_particles; i++) {
                    outputFile << "id: " << particle_filter.particles[i].id << "  x: " << particle_filter.particles[i].x << "  y: " << particle_filter.particles[i].y
                               << "  weight: " << particle_filter.particles[i].weight << std::endl;
                }

                outputFile.close();
            }
//        std::this_thread::sleep_for(std::chrono::seconds(10));

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}

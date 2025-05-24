//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "shr_utils/geometry.hpp"

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

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
#include "particle_filter/helper.hpp"
#include "particle_filter/particle_filter.hpp"


TransitionMeshHandler::TransitionMeshHandler() {
    init();
}

void TransitionMeshHandler::init() {
        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter_mesh");
        auto mesh_file = (pkg_dir / "config" / "olson_trans.obj").string();

        auto [mesh_verts, mesh_names] = shr_utils::load_meshes_squares(mesh_file);
        for (int i = 0; i < mesh_names.size(); i++) {
//            std::cout << "mesh name: " << mesh_names[i] << std::endl;
//            std::cout << "mesh vert: " << mesh_verts[i] << std::endl;
            auto name = mesh_names[i];
            auto verts = mesh_verts[i];
            mesh_vert_map_[name] = extractMinMax(verts);

        }
    }

std::string TransitionMeshHandler::get_dest(std::string aoi){
    auto it = aoi_to_dest.find(aoi);
    if (it == aoi_to_dest.end()) {
        // Des not found
        std::cout << "NO Destination for aoi" <<  aoi << std::endl;
        return "";
    }

    return it->second;
}

std::vector<float> TransitionMeshHandler::extractMinMax(const Eigen::MatrixXd& matrix) {
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

std::string TransitionMeshHandler::monitor_lndmark(double patient_x, double patient_y){
    // bounds = [x_min, x_max, y_min, y_max]

    for (const auto& pair : aoi_to_dest) {
        if ( check_person_at_loc(pair.first, patient_x, patient_y) ){
            return pair.first;
        }
    }
    return "";


    // Looping over keys using an iterator
//    for (auto it = my_map.begin(); it != my_map.end(); ++it) {
//        std::cout << "Key: " << it->first << std::endl;  // Access key using it->first
//    }
//
//    // Or using range-based for loop (C++11 and beyond)
//    for (const auto& pair : my_map) {
//        std::cout << "Key: " << pair.first << std::endl;  // Access key with pair.first
//    }
}

bool TransitionMeshHandler::check_person_at_loc(const std::string& lndmrk, double patient_x, double patient_y){
    // bounds = [x_min, x_max, y_min, y_max]
    auto it = mesh_vert_map_.find(lndmrk);
    if (it == mesh_vert_map_.end()) {
        // Landmark not found
        std::cout << "NO MESH FOUND FOR LANDMARK" <<  lndmrk << std::endl;
        return false;
    }

    std::vector<float> bounds = it->second;

    if (bounds[0] < patient_x && patient_x < bounds[1] &&
        bounds[2] < patient_y && patient_y < bounds[3]) {
        return true;
    }

    return false;
}

void TransitionMeshHandler::sample_in_bounds(const std::string& destination_area, std::vector<Particle> &particles){
    std::vector<float> bounds_dest = mesh_vert_map_[destination_area];

    std::random_device rd;  // Seed
    std::mt19937 gen(rd()); // Random number generator
    std::uniform_real_distribution<float> dist_x(bounds_dest[0], bounds_dest[1]);
    std::uniform_real_distribution<float> dist_y(bounds_dest[2], bounds_dest[3]);
//    std::cout << "GEN: " << std::endl;

    for (auto &p: particles) {
        // Generate random values within the bounds
        p.x = static_cast<double>(dist_x(gen));
        p.y =  static_cast<double>(dist_y(gen));
    }
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


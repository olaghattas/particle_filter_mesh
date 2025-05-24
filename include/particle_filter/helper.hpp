//
// Created by olagh48652 on 3/2/25.
//

#ifndef BUILD_HELPER_H
#define BUILD_HELPER_H

#include <assimp/postprocess.h>
#include <filesystem>
#include <iostream>
#include <array>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <map>
#include <Eigen/Dense>
#include "shr_utils/geometry.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
//#include "particle_filter/particle_filter.hpp"

// todo : have it declared in particle_filter
struct Particle {
    int id;
    double x;
    double y;
    double z;
    double theta;
    double weight;
};

class TransitionMeshHandler {

public:
    // constructor
    TransitionMeshHandler();
    // destructor
    ~TransitionMeshHandler() = default;

    void init();
    std::vector<float> extractMinMax(const Eigen::MatrixXd& matrix);
    bool check_person_at_loc(const std::string& lndmrk,  double patient_x, double patient_y);
    void sample_in_bounds(const std::string& monitored_area, std::vector<Particle> &particles);
    std::string monitor_lndmark(double patient_x, double patient_y);

    std::unordered_map<std::string, std::vector<float>> mesh_vert_map_;
    // map door status index to aoi
    std::unordered_map<std::string, int> aoi_to_door = {{"indoor", 2}, {"corridor", 0}};
    std::unordered_map<std::string, int> aoi_to_ms = { {"indoor", 1},{"corridor", 0}};

    // maps interest area to dest
    // the idea is that we are interested in areas where transition can happen
    // this maps the area of intrest (aoi) to the potential location
    std::unordered_map<std::string, std::string> aoi_to_dest = {{"indoor", "outdoor"}, {"corridor", "bedroom"}};

    std::string get_dest(std::string aoi);
};

#endif //BUILD_HELPER_H

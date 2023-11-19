//
// Created by ola on 6/14/23.
//

#ifndef SMART_HOME_PARTICLE_FILTER_H
#define SMART_HOME_PARTICLE_FILTER_H

#include <math.h>
#include <vector>
#include <array>
//

#include <iostream>
#include <filesystem>
#include <Eigen/Dense>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>


struct Particle {
    int id;
    double x;
    double y;
    double z;
    double theta;
    double weight;
};

struct Observation {
    std::string name;        // Id of matching landmark. landmark in our case is the joint we are starting with one but later will include all joints
    double x;      // x position of landmark (joint) in world
    double y;      // y position of landmark (joint) in world
    double z;      // z position of landmark (joint) in world
}; // going to be 1x2 for now (left shoulder joint)

/*
     * Computes the Euclidean distance between two 2D points.
     * @param (x1,y1) x and y coordinates of first point
     * @param (x2,y2) x and y coordinates of second point
     * @output Euclidean distance between two 2D points
     */
inline double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

class ParticleFilter {
private:
    // Number of particles to draw
    int num_particles;

    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;




public:
    // map with the mesh vertices
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_;
    // Set of current particles
    std::vector<Particle> particles;

    // Constructor
    ParticleFilter(int num) : num_particles(num), is_initialized(false) {}

    // Destructor
    ~ParticleFilter() {}

    void init(std::pair<double, double> x, std::pair<double, double> y, std::pair<double, double> z,
              std::pair<double, double> theta);

    void motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                      std::vector<bool> doors_status);

    void updateWeights(double std_landmark[],
                       std::vector<Observation> observations,
                       Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams);

    void resample();

    /**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
    bool initialized() const {
        return is_initialized;
    }

    void enforce_non_collision(const std::vector <Particle> &old_particles,
                                               std::vector<bool> doors_status);
    bool check_particle_at(const std::string &loc, Eigen::Vector3d point);
    void write_to_file(std::string filename);
    float sample(float mean, float variance);

};


#endif //SMART_HOME_PARTICLE_FILTER_H

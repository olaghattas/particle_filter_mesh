//
// Created by ola on 6/14/23.
//

#ifndef SMART_HOME_PARTICLE_FILTER_H
#define SMART_HOME_PARTICLE_FILTER_H

#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <filesystem>
#include <Eigen/Dense>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "particle_filter/helper.hpp"

enum PersonState {
    UNSEEN,  // Unseen but person still at home
    BEDROOM,
    OUTDOOR,
    FACE_RECOGNIZED
};

struct Observation {
    std::string name;        // Id of matching landmark. landmark in our case is the joint we are starting with one but later will include all joints
    double x;      // x position of landmark (joint) in world
    double y;      // y position of landmark (joint) in world
    double z;      // z position of landmark (joint) in world
    bool des_pers; // yes if the observation is for the person (based on face recognition)
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


    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;

    TransitionMeshHandler transition_mesh_handler;
    bool monitoring_flag = false;
    bool obs_during_monitoring = false;
    bool no_obs_during_monitoring = false;
    bool door_of_int_open = false;
    bool ms_of_int_triggered = false;
    // Random number generator
//    std::random_device rd;
//    std::mt19937 gen;

public:

    std::vector<Particle> initial_part_dist;
    double patient_x = std::nan("");
    double patient_y = std::nan("");

    // Check if start_time is NaN (we use min time as an equivalent check)
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::time_point::min();

    std::string monitoring = "";
    // Number of particles to draw
    int num_particles;
    bool no_readings = true;
    // map with the mesh vertices
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_;
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_room;
    std::unordered_map<std::string, Eigen::MatrixXd> view_points_mesh_vert_map_;

    // Set of current particles
    std::vector<Particle> particles;
    std::string prev_camera_name = "";
    std::string curr_camera_name = "";
    //Store previous observations
//    Eigen::Vector2d previous_observation;
//    Eigen::Vector2d current_observation;
    int previous_count;
    Eigen::Vector2d avg_displacement;

    // use location where the most particles are at
    // this is after the particles with no observation start spreading and some particles end up
    // in different rooms (stochasticity/noise). due to the weights not updating with no observations it might choose particles
    // with lesser number of particles
    // another approach is resampling when no observation based on number of particles
    bool use_max_loc;
    std::string max_particles_loc;
    std::pair<double, double> current_observation_;
    std::pair<double, double> previous_observation_;

    // Constructor
    // gen: generates raw random numbers that can be passed into a distribution function to obtain random samples.

//    ParticleFilter(int num) : num_particles(num), is_initialized(false) , gen(rd()) {}
    ParticleFilter(int num) : num_particles(num), is_initialized(false) {}

    // Destructor
    ~ParticleFilter() = default;

    void check_spread();
    void add_noise(double std_dev);

    void init(std::pair<double, double> x, std::pair<double, double> y, std::pair<double, double> z,
              std::pair<double, double> theta);

//    void motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
//                      std::vector<bool> doors_status, std::string observation);

    void motion_model_noisy(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                            const std::vector<bool> &doors_status, const std::string &observation, PersonState& person_state, const std::vector<bool> &ms_status);

//    void updateWeights(double std_landmark[],
//                       std::vector<Observation> observations,
//                       Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams);

    void updateWeightsWithoutObs(double std_landmark[]);

    void updateWeightsWithObs(double std_landmark[],
                       std::vector<Observation> observations,
                       Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams);
    void resample();
    void residual_resample();
    double calculateNeff();
    void check_unique_particles();

    void normalize_weights(double sum);
    std::string find_landmark_with_most_particles();

    /**
     * initialized Returns whether particle filter is initialized yet or not.
     */
    bool initialized() const {
        return is_initialized;
    }

    void enforce_non_collision(const std::vector <Particle> &old_particles,
                                               const std::vector<bool> &doors_status, const std::string &observation);
    bool check_particle_at(const std::string &loc, Eigen::Vector3d point);
    bool check_particle_room(const std::string &loc, Eigen::Vector3d point);
    bool check_particle_at_cam_view(const std::string &loc, Eigen::Vector3d point);
    void write_to_file(std::string filename);
    float sample(float mean, float variance);
    void particles_in_range(std::pair<double, double> x_bound, std::pair<double, double> y_bound, int ind_start);
//    void special_transitions(std::vector<bool> doors_status);

    void special_transitions(const std::vector<bool> &doors_status, PersonState & person_state, const std::vector<bool> &ms_status);


    };


#endif //SMART_HOME_PARTICLE_FILTER_H

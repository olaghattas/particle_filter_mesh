//
// Created by ola on 6/14/23.
//
#include <random>
#include <algorithm>
#include <map>
#include <numeric>
#include "particle_filter/particle_filter.hpp"
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
// needed for the projection
#include <iostream>
#include "shr_utils/geometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <ctime>
#include <set>


#define EPSILON 1e-4
void ParticleFilter::check_unique_particles() {
    std::set<std::pair<double, double>> unique_positions;

    for (const auto& p : particles) {
        unique_positions.insert({p.x, p.y});
    }

//    std::cout << "Unique particles: " << unique_positions.size() << " out of " << num_particles << std::endl;

    if (unique_positions.size() < num_particles * 0.5) { // Less than 50% unique
//        std::cout << "Warning: Particles have lost diversity!" << std::endl;
    }
}

void ParticleFilter::normalize_weights(double sum_weights) {

    // Only Compute the sum of all particle weights when its not passed
    if (std::isnan(sum_weights)) {
        sum_weights = 0.0;
        for (const Particle &particle: particles) {
            sum_weights += particle.weight;
        }
    }

    // Normalize the weights so that they sum up to one
    if (sum_weights == 0) {
        // Handle edge case (e.g., all weights are zero)
        for (auto& p : particles) p.weight = 1.0 / num_particles;
    } else {
        for (auto& p : particles) p.weight /= sum_weights;
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
//        std::cout << "NO LANDMARK" << std::endl;
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


    // TODO: find more areas to initialize particles
    avg_displacement(0.0, 0.0);
//    previous_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
//    current_observation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    previous_observation_ = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    current_observation_ = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};

    previous_count = 0;

    // Add random Gaussian noise to each particle.

    std::random_device rd;
    std::mt19937 gen;

    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::uniform_real_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter_mesh");
    particles.clear();
    weights.clear();


    // used write_to_file function ot save the distribution of the particles and used it to initialize
    // this ensures better coverage and easier to be done
    std::string filename = (pkg_dir / "config" / "initial_dist.txt").string();
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
//        std::cout << "line:  " << line << std::endl;
        std::istringstream iss(line);
//        std::cout << "line:  " << line << std::endl;

        Particle p;

        // Extract data using the file format
        std::string id_str, x_str, y_str, weight_str;
        std::string id_str_, x_str_, y_str_, weight_str_;
        iss >> id_str_ >> id_str >> x_str_ >> x_str >> y_str_ >> y_str >> weight_str_ >> weight_str;

        // Parse numerical values from the formatted strings
        p.id = std::stoi(id_str); // Skip "id:"
        p.x = std::stof(x_str); // Skip "x:"
        p.y = std::stof(y_str); // Skip "y:"
        p.theta =  yawNoise(gen);
        p.weight = 1.0;

//        // Add the particle and its weight
        particles.push_back(p);
        weights.push_back(p.weight);
    }

    file.close();
    normalize_weights(std::nan(""));
    initial_part_dist = particles;
//    write_to_file("first_run.txt");
    is_initialized = true;



    auto mesh_file = (pkg_dir / "config" / "gaskin_collision_mesh.obj").string();

    auto [mesh_verts, mesh_names] = shr_utils::load_meshes(mesh_file);
    for (int i = 0; i < mesh_names.size(); i++) {
        auto name = mesh_names[i];
        auto verts = mesh_verts[i];
        mesh_vert_map_[name] = verts;
    }

    auto view_points_mesh_file = (pkg_dir / "config" / "cam_view_gaskin.obj").string();

    auto [view_points_mesh_verts, view_points_mesh_names] = shr_utils::load_meshes(view_points_mesh_file);
    for (int i = 0; i < view_points_mesh_names.size(); i++) {
        auto name_mesh = view_points_mesh_names[i];
        auto verts_mesh = view_points_mesh_verts[i];
        view_points_mesh_vert_map_[name_mesh] = verts_mesh;
    }

    auto room_mesh_file = (pkg_dir / "config" / "gaskin_person_mesh.obj").string();

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
    std::mt19937 gen;

    for (int i = ind_start; i < ind_start + 10; ++i) {
        particles[i].x = xNoise(gen);
        particles[i].y = yNoise(gen);

    }
}

void ParticleFilter::motion_model_noisy(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                                       const std::vector<bool> &doors_status, const std::string &observation) {

    std::normal_distribution<double> xNoise(0, 0.15);
    std::normal_distribution<double> yNoise(0, 0.15);
    std::normal_distribution<double> zNoise(0, 0.03);
    std::normal_distribution<double> yawNoise(0, 0.03);


    auto particles_before = particles;
//    std::cout << "before p.x " << particles[0].x << std::endl;
    std::random_device rd;
    std::mt19937 gen;
    for (auto &p: particles) {

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
//    std::cout << "after p.x " << particles[0].x << std::endl;

    enforce_non_collision(particles_before, doors_status, observation);
    // check for speacial cases
}

// Function to calculate Neff
// This metric ensures the particle filter maintains diversity and avoids particle depletion.
// If all particles have equal weights, Neff=N, meaning all particles are equally contributing.
double ParticleFilter::calculateNeff() {
    double sum_squared = 0.0;
    double w;
    // Sum of squared weights
    for (int i = 0; i < num_particles; ++i) {
        w = particles[i].weight;
        sum_squared += w * w;
    }

    // Return the effective number of particles
    return 1.0 / sum_squared;
}

float ParticleFilter::sample(float mean, float variance) {
    // randomly sample from a Normal distribution
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<> dist(mean, sqrt(variance));
    return dist(gen);
}

void ParticleFilter::resample() {
    std::random_device rd;
    std::mt19937 gen;
    // Low-variance resampler
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles);

//    std::vector<Particle> resampled_particles(num_particles);
    std::vector<Particle> resampled_particles = particles;

    double c = particles[0].weight;; // Cumulative weight
    int i = 0; // Index for the original particles
    float r = double(dist(gen)); // Random starting point

    for (int m = 0; m < num_particles; m++) {
        float u = r + (float) m / num_particles;

        // Find the particle whose cumulative weight satisfies u
        while (u > c && i < num_particles) {
            i++;
            c += particles[i].weight;
        }
        resampled_particles[m].x = particles[i].x;
        resampled_particles[m].y = particles[i].y;
        resampled_particles[m].weight =  1.0 / num_particles; // Reset weight

    }

    particles = resampled_particles;

    // prevent particle collapse
//    add_noise(0.1);
}

//If you detect a collapse, you can add small random noise to maintain diversity:
void ParticleFilter::add_noise(double std_dev) {
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> noise(0.0, std_dev);

    for (auto& p : particles) {
        p.x += noise(gen);
        p.y += noise(gen);
    }
}

//If the variance is too small, particles are collapsing.
void ParticleFilter::check_spread() {
    double mean_x = 0.0, mean_y = 0.0;
    double var_x = 0.0, var_y = 0.0;

    for (const auto& p : particles) {
        mean_x += p.x;
        mean_y += p.y;
    }
    mean_x /= num_particles;
    mean_y /= num_particles;

    for (const auto& p : particles) {
        var_x += (p.x - mean_x) * (p.x - mean_x);
        var_y += (p.y - mean_y) * (p.y - mean_y);
    }
    var_x /= num_particles;
    var_y /= num_particles;

    //std::cout << "Variance in X: " << var_x << ", Variance in Y: " << var_y << std::endl;

    if (var_x < 1e-3 && var_y < 1e-3) {
        std::cout << "Warning: Particles have collapsed to a single location!" << std::endl;
    }
}

void ParticleFilter::updateWeightsWithObs(double std_landmark[],
                                          std::vector<Observation> observations,
                                          Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams) {

    max_particles_loc = "";
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_z = std_landmark[2];
    double weights_sum = 0;

//    if (!current_observation.hasNaN()) {
    previous_observation_ = current_observation_;
//    }

    // if there is an observation update the particle near the observation

    Observation current_obs = observations[0]; // TODO be changed when more observations are added
    Eigen::Vector4d homogeneousPoint;
    homogeneousPoint << current_obs.x, current_obs.y, current_obs.z, 1.0;

    Eigen::Vector4d TransformedPoint;

    TransformedPoint <<
                     extrinsicParams(0, 0) * homogeneousPoint[0] + extrinsicParams(0, 1) * homogeneousPoint[1] +
                     extrinsicParams(0, 2) * homogeneousPoint[2] + extrinsicParams(0, 3) * homogeneousPoint[3],
            extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
            extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
            extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
            extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
            extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
            extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];


    /// ONLY ONE OBSERVATION AT A TIME
    current_observation_ = {TransformedPoint[0], TransformedPoint[1]};

    const double gaussian_norm = 1.0 / (2 * M_PI * sigma_x * sigma_y);

    // loop through each of the particle to update
    for (int i = 0; i < num_particles; ++i) {
        Particle *p = &particles[i];

        double dx = p->x - TransformedPoint[0];
        double dy = p->y - TransformedPoint[1];
        double factor = 4;

        // Dynamically compute sigma based on the order of magnitude of x_ and y_
//        sigma_x = std::pow(10, std::floor(std::log10(std::abs(x_))) - 1); // Order of magnitude for x_
//        sigma_y = std::pow(10, std::floor(std::log10(std::abs(y_))) - 1);

//        double gaussian = (std::pow(x_, 2) / (2 * factor * std::pow(sigma_x, 2))) +
//                          (std::pow(y_, 2) / (2 * std::pow(sigma_y, 2)));

        double exponent = (dx * dx) / (2 * sigma_x * sigma_x)
                          + (dy * dy) / (2 * sigma_y * sigma_y);

//        double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y);
//        gaussian = exp(-gaussian);
        // Avoid numerical underflow for small exponents
        double weight = gaussian_norm * exp(-exponent);
//        gaussian = gaussian * gaussian_factor;

        weights_sum += weight;
//        p->weight = weight;
        particles[i].weight = weight;
    }

    // Normalize weights
    normalize_weights(weights_sum);


}

void ParticleFilter::updateWeightsWithoutObs(double std_landmark[]) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double weights_sum = 0;
    previous_observation_ = current_observation_;
    // if there is an observation update the particle near the observation
    current_observation_ = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    for (int i = 0; i < num_particles; ++i) {
        Eigen::Vector3d point = {particles[i].x, particles[i].y, -0.5};

        // Decrease weight of particle in cam view
        if (check_particle_at_cam_view("cam_view", point)) {
            // TODO: check diff weights
            particles[i].weight = 0;
            //  particles[i].weight = particles[i].weight / 10;
        }
        weights_sum +=  particles[i].weight;
    }

    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        particles[i].weight /= weights_sum;
    }

    max_particles_loc = find_landmark_with_most_particles();
    std::cout << "max_loc _ " << max_particles_loc << std::endl;


}

bool ParticleFilter::check_particle_at(const std::string &loc, Eigen::Vector3d point) {
    // collision mesh
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
                                           const std::vector<bool> &doors_status, const std::string &observation) {


//    std::vector<bool> getdoorstatus() {
//        // TRUE for closed and False for open
//        // should align with patrticle filter enforce collision landmarks orderc
////        bedroom_door, bathroom_door, living_room_door, outside_door
//        return {door_main, door_bedroom, door_trash, door_back};
//    }
//
//    std::vector<bool> getmsstatus() {
//        // TRUE for closed and False for open
//        return {ms_bedroom, ms_trash};
//    }

    // LANDMARK ORDER SHOULD MATCH DOOR STATUS ORDER
    std::vector<std::string>
            lndmarks = {"obstacles", "main_door", "bedroom_door", "trash_door","backdoor"};

    std::vector<std::string> view_point = {"cam_view"};

    for (int i = 0; i < num_particles; ++i) {
        Eigen::Vector3d point = {particles[i].x, particles[i].y, -0.5};

        // ###### COLLISIONS WITH OBSTACLES ########
        if (check_particle_at(lndmarks[0], point)) {
            // obstacle (not door)
            particles[i] = old_particles[i];
            particles[i].weight = 0.0;

        } else if (check_particle_at(lndmarks[1], point)) {
            // bedroom_door
            // the index should correspond to the door in door status found in article_filter_node.cpp
            if (doors_status[0]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[2], point)) {
            // bathroom
            // the index should correspond to the door in door status found in article_filter_node.cpp
            if (doors_status[1]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        } else if (check_particle_at(lndmarks[3], point)) {
            // main door
            // the index should correspond to the door in door status found in article_filter_node.cpp
            if (doors_status[2]) {
                // door 2 closed keep old particles
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        }else if (check_particle_at(lndmarks[4], point)) {
            if (doors_status[3]) {
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        }
                // ###### POINTS GOING INTO CAMERA VIEW POINT WHEN NO PERSON IS THERE ########
            // doesnt allow the particle to go into view points when no observation in camera

        else if (observation.empty() && check_particle_at_cam_view(view_point[0], point)) {
            // update only if particle was not already in cam view
            if (!check_particle_at_cam_view(view_point[0], {old_particles[i].x, old_particles[i].y, -0.5})) {
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }

        }
    }
}

void  ParticleFilter::reset_monitoringDetails(){
    monitoring_flag = false;
    monitoring_details.monitored_area = "";
    monitoring_details.trigger_sensor = "";
    monitoring_details.sensor_already_triggered=false;
    monitoring_details.start_time = std::chrono::steady_clock::time_point();  // Reset to epoch
}

std::chrono::steady_clock::time_point getCurrentTime() {
    return std::chrono::steady_clock::now();
}

void  ParticleFilter::apply_special_transitions(const std::vector<bool> &doors_status, PersonState &person_state, const std::vector<bool> &ms_status){
    // if not monitoring then dont do anything
    if (!monitoring_flag) return;


    auto now = getCurrentTime();
    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - monitoring_details.start_time);

    if (elapsed >= std::chrono::minutes(2)){
        reset_monitoringDetails();
        monitoring_flag = false;
        return;
    }

    bool sensor_triggered = false;
    // check if sensor of interest triggered
    if (monitoring_details.sensor_already_triggered){
        sensor_triggered = true;
    }
    else{
        if (monitoring_details.trigger_sensor == "ms"){
            int ms_index = transition_mesh_handler.aoi_to_ms[monitoring_details.monitored_area];
            sensor_triggered = ms_status[ms_index];
        }else{
            int door_index = transition_mesh_handler.aoi_to_door[monitoring_details.monitored_area];
            sensor_triggered = !doors_status[door_index];
        }
    }

    if (sensor_triggered){
        // transition
        std::string dest = transition_mesh_handler.aoi_to_dest[monitoring_details.monitored_area];
        if (dest == "bedroom_outside"){
            person_state = BEDROOM;
        }else {
            person_state = OUTDOOR;
        }

        transition_mesh_handler.sample_in_bounds(dest, particles);
        monitoring_flag = false;
        reset_monitoringDetails();
        return;
    }

}


void  ParticleFilter::special_transitions_monitoring(const std::vector<bool> &doors_status, const std::vector<bool> &ms_status, bool person_doorway){
    // will be called only if there is an observation
    if (isnan(patient_x) && isnan(patient_y)) return;

    if (monitoring_details.monitored_area.empty()){
        std::cout << "Person not currently being monitored. Checking special locations..." << std::endl;
        monitoring_details.monitored_area = transition_mesh_handler.monitor_lndmark(patient_x, patient_y);

        // overwrite if person_doorway is true
        // from how getObservation works n=unless there are no observation this will not be triggered
        if(person_doorway){
            monitoring_details.monitored_area = "main_inside";
            monitoring_details.trigger_sensor = "ds";
        }

        if (!monitoring_details.monitored_area.empty()){
            std::cout << "Monitoring started for area: " << monitoring_details.monitored_area << std::endl;
            monitoring_flag = true;
            monitoring_details.start_time = getCurrentTime();

            if (monitoring_details.trigger_sensor.empty()){
                // todo optimize later
                if (monitoring_details.monitored_area == "bedroom_inside" || monitoring_details.monitored_area == "trash_inside") {
                    monitoring_details.trigger_sensor = "ms";
                } else {
                    monitoring_details.trigger_sensor = "ds";
                }
            }

        }
    }

    if (monitoring_flag) {
        // special case since it depends on topic; if the topic turns false by default this will not be triggered
        if (monitoring_details.monitored_area != "main_inside"){
            if (!transition_mesh_handler.check_person_at_loc(monitoring_details.monitored_area, patient_x, patient_y)) {
    //          std::cout << "Person left special area, exiting monitoring." << std::endl;
                monitoring_flag = false;
                reset_monitoringDetails();
                return;
            }
        }

        if (!monitoring_details.sensor_already_triggered) {
            if (monitoring_details.trigger_sensor == "ms") {
                int ms_index = transition_mesh_handler.aoi_to_ms[monitoring_details.monitored_area];
                monitoring_details.sensor_already_triggered = ms_status[ms_index];
            } else {
                int door_index = transition_mesh_handler.aoi_to_door[monitoring_details.monitored_area];
                monitoring_details.sensor_already_triggered = !doors_status[door_index];
            }
        }
    }
}


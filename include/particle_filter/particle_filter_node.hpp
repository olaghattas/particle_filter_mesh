//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
//#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "particle_filter_msgs/msg/pose_msg.hpp"
#include "zed_interfaces/msg/bounding_box3_d.hpp"
#include "zed_interfaces/msg/object.hpp"
#include <cstdlib>

class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_3d_pt;

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;

    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_lv;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_bd;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr person_doorway;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lv_label_H; // H is the person of interest lv corresponds to living_room (cam location)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bd_label_H;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lv_label_F; // other person in the house
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bd_label_F;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_main_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_bedroom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_trash_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_back_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ms_bedroom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ms_trash_sub;

    // Observation observation; // Member variable to store the observation

    // to prevent overriding
    // Member variable to store the observation
    Observation observation_living;
    Observation observation_backdoor;
    //topic
    Observation observation_main_door;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    bool door_main;
    bool door_bedroom;
    bool door_back;
    bool door_trash;
    bool ms_bedroom;
    bool ms_trash;


    std::vector<int> lv_label_h;
    std::vector<int> bd_label_h;


    std::vector<int> lv_label_f;
    std::vector<int> bd_label_f;

// if true then the labels of howie and suzie will nbe flipped
// for debug
    bool f_is_h = false;

public:
    PersonState currentStateH;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_person_loc;
    bool first_obs = false;

    ParticleFilterNode() : rclcpp::Node("particle_filter"), currentStateH(UNSEEN)  {

        publish_person_loc = this->create_publisher<std_msgs::msg::Float64MultiArray>("person_loc", 10);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_sub_lv = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_living_room/zed_node_living_room/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_living_room(msg); });

        pose_sub_bd = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_back_door/zed_node_back_door/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_backdoor(msg); });


        door_main_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_main_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorMaindoorCallback(msg); });

        door_bedroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_bedroom_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBedroomCallback(msg); });

        door_trash_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_trash_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorTrashCallback(msg); });

        door_back_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_back_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBackCallback(msg); });

        ms_bedroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_motion_bedroom", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { MSBedroomCallback(msg); });

        ms_trash_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_motion_trash", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { MSTrashCallback(msg); });

        person_doorway = create_subscription<std_msgs::msg::Bool>(
                "/person_at_doorway", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { PersonDoorwayCallback(msg); });

        lv_label_H = create_subscription<std_msgs::msg::Int32>(
                "living_room_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_Callback(msg); });

        bd_label_H = create_subscription<std_msgs::msg::Int32>(
                "/back_door_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { bd_label_Callback(msg); });

        lv_label_F = create_subscription<std_msgs::msg::Int32>(
                "/living_room_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_f_Callback(msg); });

        bd_label_F = create_subscription<std_msgs::msg::Int32>(
                "/back_door_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { bd_label_f_Callback(msg); });

    }

    void MSBedroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        ms_bedroom = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "ms_bedroom ->open;" << ms_bedroom << std::endl;
    }

    void MSTrashCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        ms_trash = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "ms_corridor ->open;" << ms_corridor << std::endl;
    }

    void lv_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
//        lv_label_h.push_back(msg->data);
        if (f_is_h){
            lv_label_f.push_back(msg->data);
        }else{
            lv_label_h.push_back(msg->data);
        }
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "lv_label_h " << lv_label_h << std::endl;
    }

    void bd_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
//        dw_label_h.push_back(msg->data);
        if (f_is_h){
            bd_label_f.push_back(msg->data);}
        else{
            bd_label_h.push_back(msg->data);
        }

//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "dw_label_h " << dw_label_h << std::endl;
    }

    void lv_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
//        lv_label_f.push_back(msg->data);
        if (f_is_h){
            lv_label_h.push_back(msg->data);}
        else{
            lv_label_f.push_back(msg->data);
        }
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "lv_label_f " << lv_label_f << std::endl;
    }

    void bd_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
//        dw_label_f.push_back(msg->data);
        if (f_is_h){
            bd_label_h.push_back(msg->data);}
        else{
            bd_label_f.push_back(msg->data);
        }
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "dw_label_f " << dw_label_f << std::endl;
    }

    // in unity coordinates
    const std::unordered_map<std::string, std::tuple<double, double, double>> coordinate_map = {
            {"living_room", {-0.73, 0.26, 0.0}},  // x, y, z coordinates
            {"bedroom",     {-2.94, 4.4,  0.0}},
            {"outside",     {5.98, 1.1,  0.0}},
            {"main", {3.8, 1.35, 0.0}}
    };

    std::array<double, 4> sigma_pos;

    void DoorMaindoorCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        door_main = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "doorstats->open;" << door_outdoor << std::endl;
    }

    void DoorBedroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << "********************************" << std::endl;
        door_bedroom = msg->data;
//        std::cout << "bedroom msg->open;" << msg->data << std::endl;
//        std::cout << "bedoroom doorstats->open;" << door_bedroom << std::endl;
    }

    void DoorTrashCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << "9999999999999999999999999999999999999" << std::endl;
        door_trash = msg->data;
//        std::cout << "bsth msg->open;" << msg->data << std::endl;
//        std::cout << "bedbathoroom doorstats->open;" << door_bathroom << std::endl;
    }

    void DoorBackCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << "9999999999999999999999999999999999999" << std::endl;
        door_back = msg->data;
//        std::cout << "bsth msg->open;" << msg->data << std::endl;
//        std::cout << "bedbathoroom doorstats->open;" << door_bathroom << std::endl;
    }

    std::vector<bool> getdoorstatus() {
        // TRUE for closed and False for open
        // should align with patrticle filter enforce collision landmarks orderc
//        bedroom_door, bathroom_door, living_room_door, outside_door
        return {door_main, door_bedroom, door_trash, door_back};
    }

    std::vector<bool> getmsstatus() {
        // TRUE for closed and False for open
        return {ms_bedroom, ms_trash};
    }

    Observation getObservation(ParticleFilter& particle_filter) {

        // check which state the person is in
        // state1: h face recognized take the reading
        // if previously the person was out or bedroom or unseen
        // disperse the particles so that particle would show up in the needed area
        // actually when face is not recognized we should reintializa

        // check if person is detected
        // if person state was unseen ten disperse particles so pf can pick it up
        if (observation_backdoor.des_pers || observation_living.des_pers || observation_main_door.des_pers){
            // we have an observation of h
            //        if (currentStateH == OUTDOOR || currentStateH == BEDROOM ){
            // check which one is better
            first_obs = true;
            if (currentStateH != FACE_RECOGNIZED ){
                particle_filter.particles = particle_filter.initial_part_dist;
            }
            currentStateH = FACE_RECOGNIZED;
            if (observation_backdoor.des_pers) {
                std::cout << "observation_backdoor.des_pers" << observation_backdoor.des_pers << std::endl;
                return observation_backdoor;
            }
            if (observation_living.des_pers) {
                return observation_living;
            }
            if (observation_main_door.des_pers) {
                return observation_main_door;
            }

        }
        Observation selected_observation;
        selected_observation.name = "";

        double distance_to_prev_obs = std::numeric_limits<double>::infinity();
        // state2: h left to bedroom or outside
        // we shouldnt take the reading unless his face was recognized that he is back
        if (currentStateH == OUTDOOR || currentStateH == BEDROOM ){
            return selected_observation;
        }

        if (selected_observation.name.empty()){
            currentStateH = UNSEEN;
        }
        return selected_observation;

    }

    void PersonDoorwayCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
        observation_main_door.name = "";
        observation_main_door.des_pers = false;  // Flag to indicate if it's person h

        if (msg->data){
            observation_main_door.name = "main_door";
            observation_main_door.des_pers = true;
            observation_main_door.x = 3.8;
            observation_main_door.y = 1.35;
            observation_main_door.z = 0;
        }

        return;
    }


    // TODO observation from topic
    void PosePixCallback_living_room(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "living_room", lv_label_h, lv_label_f, observation_living);
    }

    void PosePixCallback_backdoor(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "back_door", bd_label_h, bd_label_f, observation_backdoor);
    }

    // have label_h and label_f as dictionaries
    void PosePixCallback_generic(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg, const std::string location, std::vector<int> &label_h, std::vector<int> &label_f, Observation& obs) {
        // Reset observation
        obs.name = "";
        obs.des_pers = false;  // Flag to indicate if it's person h

        if (msg->objects.empty()) {
            // Clear all labels if no objects are present
            label_h.clear();
            label_f.clear();
            return;  // No objects to process
        }

        // Gets all labels found in skeleton
        std::unordered_set<int> object_labels;
        for (const auto &obj : msg->objects) {
            object_labels.insert(obj.label_id);
        }

        // Filter label_h to remove IDs not present in the skeleton
        // std::remove if gets the id nto in skeleton and erase removes them from label_h
        label_h.erase(std::remove_if(label_h.begin(), label_h.end(),
                                     [&](int id) { return object_labels.find(id) == object_labels.end(); }),
                      label_h.end());
        // Filter label_f to remove IDs not present in the skeleton
        label_f.erase(std::remove_if(label_f.begin(), label_f.end(),
                                     [&](int id) { return object_labels.find(id) == object_labels.end(); }),
                      label_f.end());

        // If label_h is not empty, select the first valid label_h
        if (!label_h.empty()) {
            int selected_label_h = label_h.front();  // First label_h
            // Process objects to find person h
            for (const auto &obj : msg->objects) {
                if (obj.label_id == selected_label_h) {
                    SetObservation(obj, true, location, obs);  // It's person h
                    return;  // Person h found, exit early
                }
            }
        }

        // Fallback: find the first object that is not in label_f
        for (const auto &obj : msg->objects) {
            if (std::find(label_f.begin(), label_f.end(), obj.label_id) == label_f.end()) {
                SetObservation(obj, false, location, obs);  // Valid fallback object (not person h)
                return;
            }
        }

        // only valid skeleton is f;  nothing to set.
        return;
    }

    // Function to set the observation based on whether it's person h or not
    void SetObservation(const zed_interfaces::msg::Object &obj, bool is_person_h, const std::string &location_name, Observation &obs) {
        obs.name = location_name;
        obs.des_pers = is_person_h;  // Flag for person h

        SetCentroidAndDimensions(obj, obs);
    }

    // Function to calculate centroid and set object dimensions
    void SetCentroidAndDimensions(const zed_interfaces::msg::Object &obj, Observation &obs) {
        zed_interfaces::msg::BoundingBox3D bounding_box = obj.bounding_box_3d;
        float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

        for (int i = 0; i < 8; ++i) {
            sum_x += bounding_box.corners[i].kp[0];
            sum_y += bounding_box.corners[i].kp[1];
            sum_z += bounding_box.corners[i].kp[2];
        }

        // Calculate centroid
        obs.x = sum_x / 8.0;
        obs.y = sum_y / 8.0;
        obs.z = sum_z / 8.0;

        // TODO: use sigma
        sigma_pos[0] = obj.dimensions_3d[0];
        sigma_pos[1] = obj.dimensions_3d[1];
        sigma_pos[2] = obj.dimensions_3d[2];
        sigma_pos[3] = 0.1;  // Default sigma value for uncertainty
    }

    void publish_3d_point(float x, float y, float z, std::string frame_id, float r, float g, float b) {
        auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        // Create a marker message for a single point
//        visualization_msgs::msg::Marker marker_msg;
        marker_msg->header.frame_id = frame_id; // Replace with your desired frame_id
        marker_msg->header.stamp = this->get_clock()->now();
        marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg->action = visualization_msgs::msg::Marker::ADD;
        marker_msg->scale.x = 0.1; // Scale factors for the point (adjust as needed)
        marker_msg->scale.y = 0.1;
        marker_msg->scale.z = 0.1;
        marker_msg->color.r = r;
        marker_msg->color.g = g;
        marker_msg->color.b = b;
        marker_msg->color.a = 1.0;

        // Set the position of the point
        marker_msg->pose.position.x = x; // Replace with your desired X coordinate
        marker_msg->pose.position.y = y; // Replace with your desired Y coordinate
        marker_msg->pose.position.z = z; // Replace with your desired Z coordinate

        // Publish the marker
//        publisher_3d_pt->publish(*marker_msg);
    }

    void publish_particles(std::vector<Particle> &particles) {       // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        // Populate the marker array with markers
        for (const auto &particle: particles) {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "unity";
            marker.header.stamp = this->get_clock()->now();
            marker.id = particle.id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = particle.x;
            marker.pose.position.y = particle.y;
            marker.pose.position.z = particle.z;
            marker.pose.orientation.z = sin(particle.theta / 2.0);
            marker.pose.orientation.w = cos(particle.theta / 2.0);
            marker.scale.x = 0.05;  // Set the scale to make the arrow thinner
            marker.scale.y = 0.01;  // Set the scale to make the arrow thinner
            marker.scale.z = 0.01;  // Set the scale to make the arrow thinner
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Add the marker to the marker array
            markerArrayMsg->markers.push_back(marker);
        }
        // Publish the marker array
        publisher_->publish(*markerArrayMsg);

    }

    void cam_extrinsics_from_tf() {

        std::vector<std::string> cams{"back_door", "living_room"};
//        std::vector<std::pair<std::string, int>> cams{"zed_kitchen_left_camera_frame"};

        // Loop over the keys of map_cam_aptag using a range-based for loop
        for (const auto &cam: cams) {

            std::cout << " cam_cam " << cam << std::endl;
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_cam_to_map = transform_tf("unity",
                                                                                     "zed_" + cam + "_cam");
            cameraextrinsics.insert(std::make_pair(cam, t_cam_to_map));

        }
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_tf(std::string toFrame, std::string fromFrame) {
        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(), std::chrono::milliseconds(100));

            geometry_msgs::msg::Transform transform_ = t.transform;

            // turn geometry transform to 4x4 matrix
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform = transform_geometry_to_matrix(transform_);
            RCLCPP_INFO(this->get_logger(), "transform %s to %s", fromFrame.c_str(), toFrame.c_str());

            return transform;

        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    fromFrame.c_str(), toFrame.c_str(), ex.what());
//            return;
        }
    }

    void stop_tf_listener(){
        if (tf_listener_) {
            tf_listener_.reset();
            tf_buffer_.reset();
            RCLCPP_INFO(this->get_logger(), "TF listener stopped.");
        }
    }

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_cam_extrinsic_matrix() {
        cam_extrinsics_from_tf();
        return cameraextrinsics;
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_geometry_to_matrix(geometry_msgs::msg::Transform transform) {
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicmatrix;
        Eigen::Quaterniond quaternion(transform.rotation.w,
                                      transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z);
        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();

        Eigen::Vector3d translationVector(transform.translation.x,
                                          transform.translation.y,
                                          transform.translation.z);

        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
        extrinsicmatrix.row(3) << 0, 0, 0, 1;
        return extrinsicmatrix;
    }

    geometry_msgs::msg::TransformStamped
    publish_transform(Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transformation_matrix, std::string frame_id,
                      std::string child_frame_id) {

        Eigen::Affine3d affine(transformation_matrix);
        Eigen::Quaterniond quaternion(affine.linear());
        Eigen::Vector3d translation(affine.translation());

        // Fill in the message
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = frame_id;
        t.child_frame_id = child_frame_id;
        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();
        t.transform.rotation.x = quaternion.x();
        t.transform.rotation.y = quaternion.y();
        t.transform.rotation.z = quaternion.z();
        t.transform.rotation.w = quaternion.w();
        return t;
    }

    geometry_msgs::msg::TransformStamped compute_mean_point(std::vector<Particle> particles) {
        //Compute the mean for all particles that have a reasonably good weight.
        //    This is not part of the particle filter algorithm but rather an
        //    addition to show the "best belief" for current position.
        double m_x = 5.0;
        double m_y = 10.0;
        double m_count = 2.0;
        int num_particles = particles.size();
        Particle best_particle;
        for (int i = 0; i < num_particles; ++i) {
            m_count += particles[i].weight;
            m_x += particles[i].x * particles[i].weight;
            m_y += particles[i].y * particles[i].weight;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = rclcpp::Clock().now();
        t.header.frame_id = "unity";
        /// should be whatever the code is expecting the name to be
        t.child_frame_id = "nathan";
        t.transform.translation.x = best_particle.x;
        t.transform.translation.y = best_particle.y;
        t.transform.translation.z = best_particle.z;
        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = sin(best_particle.theta / 2.0);
        t.transform.rotation.w = cos(best_particle.theta / 2.0);
//                std::cout << " x " << best_particle.x << " y " << best_particle.y << " z " << best_particle.z << std::endl;
        return t;
    }

    // Define a function to calculate the Euclidean distance between two points
    double euclideanDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
};

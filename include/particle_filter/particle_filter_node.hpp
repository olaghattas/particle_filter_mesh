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
#include "particle_filter_msgs/msg/pose_msg.hpp"
#include "zed_interfaces/msg/bounding_box3_d.hpp"
#include "zed_interfaces/msg/object.hpp"

#include <cstdlib>

//enum PersonState {
//    UNSEEN,  // useen but person still at home
//    BEDROOM,
//    OUTDOOR,
//    FACE_RECOGNIZED
//};

class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_3d_pt;

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;

    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_k;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_lv;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_dw;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_cor;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr k_label_H;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lv_label_H;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dw_label_H;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cor_label_H;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr k_label_F;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lv_label_F;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dw_label_F;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cor_label_F;


    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_outdoor_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_bedroom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_atelier_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ms_bedroom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ms_corr_sub;

    // Observation observation; // Member variable to store the observation

    // to prevent overriding
    // Member variable to store the observation
    Observation observation_kitchen;
    Observation observation_living;
    Observation observation_doorway;
    Observation observation_corridor;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::map<std::string, std::string> map_cam_aptag;
//    std::map<std::string, std::string> map_cam_aptag_un;

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_bedroom;
    bool door_bathroom;
    bool ms_bedroom;
    bool ms_corridor;

    std::vector<int> k_label_h;
    std::vector<int> lv_label_h;
    std::vector<int> dw_label_h;
    std::vector<int> coor_label_h;

    std::vector<int> k_label_f;
    std::vector<int> lv_label_f;
    std::vector<int> dw_label_f;
    std::vector<int> coor_label_f;



public:
    PersonState currentStateH;
    bool first_obs = false;
    ParticleFilterNode() : rclcpp::Node("particle_filter"), currentStateH(UNSEEN)  {

//        map_cam_aptag["doorway"] = "tag_" + std::string(std::getenv("tag_doorway")) + "_zed";
//        map_cam_aptag["kitchen"] = "tag_" + std::string(std::getenv("tag_kitchen")) + "_zed";
//        map_cam_aptag["dining_room"] = "tag_" + std::string(std::getenv("tag_dining_room")) + "_zed";
//
//        map_cam_aptag_un["doorway"] = "aptag_" + std::string(std::getenv("tag_doorway"));
//        map_cam_aptag_un["kitchen"] = "aptag_" + std::string(std::getenv("tag_kitchen"));
//        map_cam_aptag_un["dining_room"] = "aptag_" + std::string(std::getenv("tag_dining_room"));

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // olson
        pose_sub_k = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_kitchen/zed_node_kitchen/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_kitchen(msg); });

        pose_sub_lv = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_bedroom/zed_node_bedroom/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_living_room(msg); });


//        pose_sub_k = create_subscription<zed_interfaces::msg::ObjectsStamped>(
//                "/zed_kitchen/zed_node_kitchen/body_trk/skeletons", 1,
//                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_kitchen(msg); });
//
//        pose_sub_lv = create_subscription<zed_interfaces::msg::ObjectsStamped>(
//                "/zed_living_room/zed_node_living_room/body_trk/skeletons", 1,
//                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_living_room(msg); });

        pose_sub_dw = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_doorway/zed_node_doorway/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_doorway(msg); });

        pose_sub_cor = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed_corridor/zed_node_corridor/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_corridor(msg); });

//        ds1
        door_outdoor_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_main_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorOutdoorCallback(msg); });
        // ds2
        door_bedroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_bedroom_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBedroomCallback(msg); });
        // ds3
        door_atelier_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_atelier_door", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { DoorBathroomCallback(msg); });

        // ms1
        ms_bedroom_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_motion_bedroom", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { MSBedroomCallback(msg); });
        // ms2
        ms_corr_sub = create_subscription<std_msgs::msg::Bool>(
                "/sensors_motion_corridor", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) { MSCorridorCallback(msg); });


        // todo:  s should be h but for lab testing
        k_label_H = create_subscription<std_msgs::msg::Int32>(
                "/kitchen_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { k_label_Callback(msg); });

//        lv_label_H = create_subscription<std_msgs::msg::Int32>(
//                "/living_room_h_label", 10,
//                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_Callback(msg); });

        lv_label_H = create_subscription<std_msgs::msg::Int32>(
                "bedroom_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_Callback(msg); });


        dw_label_H = create_subscription<std_msgs::msg::Int32>(
                "/doorway_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { dw_label_Callback(msg); });

        cor_label_H = create_subscription<std_msgs::msg::Int32>(
                "/cooridor_h_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { cor_label_Callback(msg); });


        k_label_F = create_subscription<std_msgs::msg::Int32>(
                "/kitchen_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { k_label_f_Callback(msg); });

//        lv_label_F = create_subscription<std_msgs::msg::Int32>(
//                "/living_room_s_label", 10,
//                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_f_Callback(msg); });

        lv_label_F = create_subscription<std_msgs::msg::Int32>(
                "/bedroom_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { lv_label_f_Callback(msg); });

        dw_label_F = create_subscription<std_msgs::msg::Int32>(
                "/doorway_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { dw_label_f_Callback(msg); });

        cor_label_F = create_subscription<std_msgs::msg::Int32>(
                "/cooridor_s_label", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) { cor_label_f_Callback(msg); });



    }

    void MSBedroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        ms_bedroom = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "ms_bedroom ->open;" << ms_bedroom << std::endl;
    }

    void MSCorridorCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        ms_corridor = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "ms_corridor ->open;" << ms_corridor << std::endl;
    }

    void k_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        k_label_h.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "k_label_h " << k_label_h << std::endl;
    }

    void lv_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        lv_label_h.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "lv_label_h " << lv_label_h << std::endl;
    }

    void dw_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        dw_label_h.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "dw_label_h " << dw_label_h << std::endl;
    }

    void cor_label_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        coor_label_h.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "coor_label_h " << coor_label_h << std::endl;
    }

    void k_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        k_label_f.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "k_label_h " << k_label_f << std::endl;
    }

    void lv_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        lv_label_f.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "lv_label_f " << lv_label_f << std::endl;
    }

    void dw_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        dw_label_f.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "dw_label_f " << dw_label_f << std::endl;
    }

    void cor_label_f_Callback(const std_msgs::msg::Int32::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        coor_label_f.push_back(msg->data);
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "coor_label_f " << coor_label_f << std::endl;
    }

    // save coordinate map
    // need to change, these are ricks
    const std::unordered_map<std::string, std::tuple<double, double, double>> coordinate_map = {
            {"living_room", {-0.5, 0.0, 0.0}},  // x, y, z coordinates
            {"bedroom",     {-5.1, -1.7,  0.0}},
            {"outside",     {6, -0.7,  0.0}},
            {"dining_room", {1.5, 0.0, 0.0}},  // x, y, z coordinates
            {"kitchen",     {4, 0,  0.0}},
            {"bathroom",     {-4,    0.0,  0.0}},
    };

    std::array<double, 4> sigma_pos;

    void DoorOutdoorCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << " ######################################################" << std::endl;
        door_outdoor = msg->data;
//        std::cout << "msg->open;" << msg->data << std::endl;
//        std::cout << "doorstats->open;" << door_outdoor << std::endl;
    }

    void DoorBedroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << "********************************" << std::endl;
        door_bedroom = msg->data;
//        std::cout << "bedroom msg->open;" << msg->data << std::endl;
//        std::cout << "bedoroom doorstats->open;" << door_bedroom << std::endl;
    }

    void DoorBathroomCallback(const std_msgs::msg::Bool::SharedPtr &msg) {
//        std::cout << "9999999999999999999999999999999999999" << std::endl;
        door_bathroom = msg->data;
//        std::cout << "bsth msg->open;" << msg->data << std::endl;
//        std::cout << "bedbathoroom doorstats->open;" << door_bathroom << std::endl;
    }

    std::vector<bool> getdoorstatus() {
        // TRUE for closed and False for open
        // should align with patrticle filter enforce collision landmarks orderc
//        bedroom_door, bathroom_door, living_room_door, outside_door
        return {door_bedroom, door_bathroom, door_outdoor};
    }

    std::vector<bool> getmsstatus() {
        // TRUE for closed and False for open
        // should align with patrticle filter enforce collision landmarks orderc
//        bedroom_door, bathroom_door, living_room_door, outside_door
        return {ms_bedroom, ms_corridor};
    }

    Observation getObservation(ParticleFilter& particle_filter) {

        // check which state the person is in
        // state1: h face recognized take the reading
        // if previously the person was out or bedroom or unseen
        // disperse the particles so that particle would show up in the needed area
        // actually when face is not recognized we should reintializa

        // check if person is detected
        // if person state was unseen ten disperse particles so pf can pick it up
        if (observation_kitchen.des_pers || observation_doorway.des_pers || observation_living.des_pers || observation_corridor.des_pers){
            // we have an observation of h
            //        if (currentStateH == OUTDOOR || currentStateH == BEDROOM ){
            // check which one is better
            first_obs = true;
            if (currentStateH != FACE_RECOGNIZED ){
                particle_filter.particles = particle_filter.initial_part_dist;
            }
            currentStateH = FACE_RECOGNIZED;
            if (observation_kitchen.des_pers) {
                std::cout << "observation_kitchen.des_pers" << observation_kitchen.des_pers << std::endl;
                return observation_kitchen;
            }
            if (observation_doorway.des_pers) {
                return observation_doorway;
            }
            if (observation_living.des_pers) {
                std::cout << "observation_living.des_pers" << observation_living.des_pers << std::endl;
                return observation_living;
            }
            if (observation_corridor.des_pers) {
                return observation_corridor;
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


        // state 3 h is picked up but not recognized
        // choose location closest ot prev obse
//        auto prev = particle_filter.previous_observation_;

        // in map frame so i need to transform obs to map frame then use them
//        if (!observation_doorway.name.empty() || !observation_kitchen.name.empty()  || !observation_living.name.empty()  || !observation_corridor.name.empty()) {
            // there is an observation


//        if (!std::isnan(prev.first) && !std::isnan(prev.second)) {
//            // actually have first observation be recognized person
//            // first_obs = true;
//            Eigen::Vector4d TransformedPoint;
//            Eigen::Vector4d homogeneousPoint;
//            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams;
//
//            if (observation_doorway.name != "") {
////                euclideanDistance(double x1, double y1, double x2, double y2)
//                // transform then get euclidean dist to prev
//                homogeneousPoint << observation_doorway.x, observation_doorway.y, observation_doorway.z, 1.0;
//                extrinsicParams = cameraextrinsics[observation_doorway.name];
//
//                TransformedPoint <<
//                                 extrinsicParams(0, 0) * homogeneousPoint[0] +
//                                 extrinsicParams(0, 1) * homogeneousPoint[1] +
//                                 extrinsicParams(0, 2) * homogeneousPoint[2] +
//                                 extrinsicParams(0, 3) * homogeneousPoint[3],
//                        extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
//                        extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
//                        extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
//                        extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
//                        extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
//                        extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];
//
//                // no need check cause here dist is infinity
//                distance_to_prev_obs = euclideanDistance(prev.first, prev.first, TransformedPoint[0],
//                                                         TransformedPoint[1]);
//                selected_observation = observation_doorway;
//
//            }
//
//            if (observation_living.name != "") {
//
//                homogeneousPoint << observation_living.x, observation_living.y, observation_living.z, 1.0;
//
//                extrinsicParams = cameraextrinsics[observation_living.name];
//                TransformedPoint <<
//                                 extrinsicParams(0, 0) * homogeneousPoint[0] +
//                                 extrinsicParams(0, 1) * homogeneousPoint[1] +
//                                 extrinsicParams(0, 2) * homogeneousPoint[2] +
//                                 extrinsicParams(0, 3) * homogeneousPoint[3],
//                        extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
//                        extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
//                        extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
//                        extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
//                        extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
//                        extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];
//
//                if (distance_to_prev_obs >
//                    euclideanDistance(prev.first, prev.first, TransformedPoint[0], TransformedPoint[1])) {
//                    distance_to_prev_obs = euclideanDistance(prev.first, prev.first, TransformedPoint[0],
//                                                             TransformedPoint[1]);
//                    selected_observation = observation_living;
//                }
//            }
//
//            if (observation_corridor.name != "") {
//                homogeneousPoint << observation_corridor.x, observation_corridor.y, observation_corridor.z, 1.0;
//
//                extrinsicParams = cameraextrinsics[observation_corridor.name];
//                TransformedPoint <<
//                                 extrinsicParams(0, 0) * homogeneousPoint[0] +
//                                 extrinsicParams(0, 1) * homogeneousPoint[1] +
//                                 extrinsicParams(0, 2) * homogeneousPoint[2] +
//                                 extrinsicParams(0, 3) * homogeneousPoint[3],
//                        extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
//                        extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
//                        extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
//                        extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
//                        extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
//                        extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];
//                if (distance_to_prev_obs >
//                    euclideanDistance(prev.first, prev.first, TransformedPoint[0], TransformedPoint[1])) {
//                    distance_to_prev_obs = euclideanDistance(prev.first, prev.first, TransformedPoint[0],
//                                                             TransformedPoint[1]);
//                    selected_observation = observation_corridor;
//                }
//            }
//
//            if (observation_kitchen.name != "") {
//                homogeneousPoint << observation_kitchen.x, observation_kitchen.y, observation_kitchen.z, 1.0;
//
//                extrinsicParams = cameraextrinsics[observation_kitchen.name];
//                TransformedPoint <<
//                                 extrinsicParams(0, 0) * homogeneousPoint[0] +
//                                 extrinsicParams(0, 1) * homogeneousPoint[1] +
//                                 extrinsicParams(0, 2) * homogeneousPoint[2] +
//                                 extrinsicParams(0, 3) * homogeneousPoint[3],
//                        extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
//                        extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
//                        extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
//                        extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
//                        extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
//                        extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];
//                if (distance_to_prev_obs >
//                    euclideanDistance(prev.first, prev.first, TransformedPoint[0], TransformedPoint[1])) {
//                    distance_to_prev_obs = euclideanDistance(prev.first, prev.first, TransformedPoint[0],
//                                                             TransformedPoint[1]);
//                    selected_observation = observation_kitchen;
//                }
//            }
//        }


        if (selected_observation.name.empty()){

            currentStateH = UNSEEN;
        }

        return selected_observation;

    }

    void PosePixCallback_kitchen(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "kitchen", k_label_h, k_label_f, observation_kitchen);
    }

    void PosePixCallback_living_room(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "living_room", lv_label_h, lv_label_f, observation_living);
    }

    void PosePixCallback_corridor(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "corridor", coor_label_h, coor_label_f, observation_corridor);
    }

    void PosePixCallback_doorway(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        PosePixCallback_generic(msg, "doorway", dw_label_h, dw_label_f, observation_doorway);
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

        // If label_h is empty, find a valid object that is not label_f
        for (std::vector<zed_interfaces::msg::Object>::size_type ind = 0; ind < msg->objects.size(); ++ind) {
            const auto &obj = msg->objects[ind];

            // Skip objects whose labels are in label_f
            if (std::find(label_f.begin(), label_f.end(), obj.label_id) != label_f.end()) {
                continue;  // Skip Florence objects
            }

            // Fallback to the first valid object (not in label_f)
            if (fallback_ind == -1) {
                fallback_ind = ind;
                SetObservation(msg->objects[fallback_ind], false, location, obs);  // Not person h
                return;
            }
        }

        // only valid skeleton is f
        return;
    }

//    void PosePixCallback_generic(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg, const std::string location, int &label_h, int &label_f, Observation& obs) {
////        std::cout << " ************** PosePixCallback in " << location << std::endl;
////
////        std::cout << " ************** label_h  " << label_h << std::endl;
////        std::cout << " ************** label_f  " << label_f << std::endl;
//        // Reset observation
//        obs.name = "";
//        obs.des_pers = false;  // Flag to indicate if it's person h
//
//        if (msg->objects.empty()) {
//            return;  // No objects to process
//        }
//
//        // Initially assume no valid observation
//        bool found_person_h = false;
//        bool found_person_f = false;
//        bool found_valid_person = false;
//        std::vector<zed_interfaces::msg::Object>::size_type fallback_ind = -1;  // Index of the first valid object
//
//        // If both labels are empty, take the first observation
//        if (label_h == -1 && label_f == -1) {
//            SetObservation(msg->objects[0], false, location, obs);
//            return;
//        }
//
//        // Process objects to find person h or a valid object
//        for (std::vector<zed_interfaces::msg::Object>::size_type ind = 0; ind < msg->objects.size(); ++ind) {
//            const auto &obj = msg->objects[ind];
//
////            std::cout << " ************** obj.label_id  " << obj.label_id << std::endl;
//            if (obj.label_id == label_f) {
////                std::cout << " ************** obj.label_id  = label_f  " << std::endl;
//
//                found_person_f = true;
//                continue;  // Skip Florence objects
//            }
//
//            if (obj.label_id == label_h) {
//                found_person_h = true;
//                SetObservation(obj, true, location, obs);  // It's person h
//                return;  // Person H found, no need to check further
//            }
//
//            // Store the first valid object (not f or h)
//            if (!found_valid_person) {
////                std::cout << " **************  valid perosn obj.label_id  " << obj.label_id << std::endl;
//
//                fallback_ind = ind;
//                found_valid_person = true;
//            }
//        }
//
//        // Handle cases where person h or f wasn't found
//        if (label_h != -1) {
//            label_h = -1;  // Label h is no longer valid
//        }
//        if (label_f != -1 && !found_person_f) {
//            label_f = -1;  // Label f is no longer valid
//        }
//
//        // If person h was not found, fallback to the first valid object
//        if (found_valid_person) {
//            SetObservation(msg->objects[fallback_ind], false, location, obs);  // Not person h
//        }
//    }


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

//        std::vector<std::string> cams{"dining", "kitchen", "bedroom", "livingroom", "hallway", "doorway"};
        std::vector<std::string> cams{"kitchen", "doorway", "living_room", "corridor"};
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
                    tf2::TimePoint(), std::chrono::milliseconds(100000));

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

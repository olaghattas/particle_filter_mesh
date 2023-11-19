//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include <tf2_ros/transform_broadcaster.h>

#include <random>
#include <map>
#include <opencv2/opencv.hpp>
#include "detection_msgs/msg/door_status.hpp"
#include <array>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "zed_interfaces/msg/objects_stamped.hpp"
//#include "zed_interfaces/msg/bounding_box_3_d.hpp"
#include "zed_interfaces/msg/bounding_box3_d.hpp"
//#include "zed_interfaces/msg/keypoints_3_d.hpp"
#include "zed_interfaces/msg/object.hpp"
//#include "zed_interfaces/msg/skeleton_3_d.hpp"
#include "zed_interfaces/msg/pos_track_status.hpp"

//struct TransformData {
//    double posX;
//    double posY;
//    double posZ;
//    double quatX;
//    double quatY;
//    double quatZ;
//    double quatW;
//    std::string name;
//};


class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_3d_pt;
//    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr pose_sub_k;
//    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_sub_k;
    Observation observation; // Member variable to store the observation

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_livingroom;
    bool door_bedroom;
    bool door_bathroom;


public:
    ParticleFilterNode() : Node("particle_filter") {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);
//        publisher_3d_pt = this->create_publisher<visualization_msgs::msg::Marker>("d3_point_marker", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         subscribe to point coordinate info to get intrinsic parameters
//        pose_sub_k = create_subscription<std_msgs::msg::Float64MultiArray>(
//                "/body_pose_kitchen" , 1,
//                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { PosePixCallback_kitchen(msg); });

//        pose_sub_k = create_subscription<std_msgs::msg::Float64MultiArray>(
//                "/body_pose_kitchen" , 1,
//                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { PosePixCallback_kitchen(msg); });

        pose_sub_k = create_subscription<zed_interfaces::msg::ObjectsStamped>(
                "/zed2i/zed_node/body_trk/skeletons", 1,
                [this](const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) { PosePixCallback_kitchen(msg); });



        // work with one for now
//        auto pose_sub_din = create_subscription<zed_interfaces::msg::ObjectStamped>(
//                "/coord_shoulder_joint_in_px_dining", 1,
//                [this](const zed_interfaces::msg::ObjectStamped::SharedPtr msg) { PosePixCallback_dining(msg); });
//        auto pose_sub_lr = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_livingroom", 1, t_cam_to_map   0.67873 -0.102481 -0.727202  0.190389
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_livingroom(msg); });
//        auto pose_sub_hw = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_hallway", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_hallway(msg); });
//        auto pose_sub_dw = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_doorway", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_doorway(msg); });
//
//                    if (!network){
//                        // train the model
//                        network = check_collision_training(directoryPath, 3);
//                    }
        auto door_outdoor_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_outdoor", 10,
                [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorOutdoorCallback(msg); });
        auto door_livingroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_livingroom", 10,
                [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorLivingroomCallback(msg); });
        auto door_bedroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_bedroom", 10,
                [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorBedroomCallback(msg); });
        auto door_bathroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_bathroom", 10,
                [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorBathroomCallback(msg); });
    }

    std::array<double, 4> sigma_pos;

    void DoorOutdoorCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_outdoor = msg->open;
    }

    void DoorLivingroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_livingroom = msg->open;
    }

    void DoorBedroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_bedroom = msg->open;
    }

    void DoorBathroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_bathroom = msg->open;
    }

    std::vector<bool> getdoorstatus() {
       // should align with patrticle filter enforce collision lanmarks order
        return {door_bedroom, door_bathroom, door_livingroom, door_livingroom};
    }

    Observation getObservation() {
        return observation;
    }

//    void PosePixCallback_kitchen(const std_msgs::msg::Float64MultiArray::SharedPtr &msg) {
//        //# 2 -> POSE_38
//        if (msg) {
//            observation.name = "kitchen";
//            // Calculate the centroid
//            observation.x = msg->data[0];
//            observation.y = msg->data[1];
//            observation.z = msg->data[2];
//
//            sigma_pos[0] = msg->data[3];
//            sigma_pos[1] = msg->data[4];
//            sigma_pos[2] = msg->data[5];
//            sigma_pos[3] = 0.1;
//
//        } else {
//            std::cout << "no person detected" << std::endl;
//            observation.name = "";
//        }
//    }

    void PosePixCallback_kitchen(const zed_interfaces::msg::ObjectsStamped::SharedPtr &msg) {
        //# 2 -> POSE_38
        if (!msg->objects.empty()) {
//#      1 ------- 2
//#     /.        /|
//#    0 ------- 3 |
//#    | .       | |
//#    | 5.......| 6
//#    |.        |/
//#    4 ------- 7

            if (msg->objects[0].skeleton_available) {
                observation.name = "kitchen";
//                observation.x = msg->objects[0].skeleton_3d.keypoints[1].kp[0];
//                observation.y = msg->objects[0].skeleton_3d.keypoints[1].kp[1];
//                observation.z = msg->objects[0].skeleton_3d.keypoints[1].kp[2];
                zed_interfaces::msg::BoundingBox3D bounding_box = msg->objects[0].bounding_box_3d;
                float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
                for (int i = 0; i < 8; i++) {
                    sum_x += bounding_box.corners[i].kp[0];
                    sum_y += bounding_box.corners[i].kp[1];
                    sum_z += bounding_box.corners[i].kp[2];
                }

                // Calculate the centroid
                observation.x = sum_x / 8.0;
                observation.y = sum_y / 8.0;
                observation.z = sum_z / 8.0;

                sigma_pos[0] = msg->objects[0].dimensions_3d[0];
                sigma_pos[1] = msg->objects[0].dimensions_3d[1];
                sigma_pos[2] = msg->objects[0].dimensions_3d[2];
                sigma_pos[3] = 0.1;
            }
        } else {
            std::cout << "no person detected" << std::endl;
            observation.name = "";

        }
    }
//    LandmarkObs PosePixCallback_dining(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
//    LandmarkObs PosePixCallback_livingroom(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
//    LandmarkObs PosePixCallback_hallway(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
//    LandmarkObs PosePixCallback_doorway(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }

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

//            std::map<std::string, std::string> map_cam_aptag;
        // map cameras to aptags ids
//            map_cam_aptag["dining"] = "aptagcam_extrinsics_from_tf_1";
//            map_cam_aptag["kitchen"] = "aptag_2";
//            map_cam_aptag["bedroom"] = "aptag_3";
//            map_cam_aptag["living"] = "aptag_4";
//        std::vector<std::string> cams{"dining", "kitchen", "bedroom", "livingroom", "hallway", "doorway"};
        std::vector<std::string> cams{"zed2i_left_camera_frame"};

        // Loop over the keys of map_cam_aptag using a range-based for loop
        for (const auto &cam: cams) {
            // Get transformation matrix from camera to aptag /// from aptag detection
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_cam_to_aptag = transform_tf("tag_18_zed", cam);
            std::cout << " t_cam_to_aptag " << t_cam_to_aptag << std::endl;

            // Get transformation matrix from map to waptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_waptag_to_cam = transform_tf("unity", "aptag_18");
            std::cout << " t_waptag_to_cam " << t_waptag_to_cam << std::endl;

            // Get transformation matrix from map to aptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_cam_to_map = t_waptag_to_cam * t_cam_to_aptag;
//            std::cout << " t_cam_to_map " << t_cam_to_map << std::endl;
//            cameraextrinsics.insert(std::make_pair(cam, t_map_to_cam));
            cameraextrinsics.insert(std::make_pair("kitchen", t_cam_to_map));
        }
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_tf(std::string toFrame, std::string fromFrame) {

        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(), std::chrono::milliseconds(10000000));

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

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParticleFilterNode>();

    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> camera_extrinsics;

    // Todo map observation to camera intrinsic and extrinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));

    bool not_initialized = true;
    while (rclcpp::ok()) {
        /// comment when not debugging
        if (not_initialized) {
            camera_extrinsics = node->get_cam_extrinsic_matrix();

            if (camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {
            bool debug = false;
            if (debug) {
                ////////// START  TESTINGGGG  //////////
//            auto cam_ext = camera_extrinsics["kitchen"];
//                Eigen::Matrix<double, 4, 4, Eigen::RowMajor> cam_ext;
//                cam_ext << 0.640011, -0.0872212, -0.763399, 0.491301,
//                        0.618233, 0.648423, 0.444224, -6.02883,
//                        0.45626, -0.756267, 0.468921, -2.99047,
//                        0, 0, 0, 1;


//                camera_extrinsics["kitchen"]   0.570834  0.815315 0.0970102   1.15011
//                -0.744756  0.563896 -0.356876   2.46518
//                -0.34567  0.131468  0.929101 -0.213102

//                auto t_ = node->publish_transform(cam_ext, "map", "zed_cam");
//                tf_broadcaster_->sendTransform(t_);
                // test point in camera frame
                // - 0.6684114336967468
                //      - -0.837434709072113
                //      - -0.44801023602485657
                // visualize pt in blue

//                float x = 0.668;
//                float y = -0.83;
//                float z = -0.448;
//                node->publish_3d_point(x, y, z, "zed_cam", 0, 0, 1);
//
//                std::cout << " camera_extrinsic " << cam_ext << std::endl;
//
//                Eigen::Vector4d homogeneousPoint;
//                homogeneousPoint << x, y, z, 1.0;
//                std::cout << " homogeneousPoint " << homogeneousPoint << std::endl;
//
//                Eigen::Vector4d TransformedPoint = cam_ext * homogeneousPoint;
//
//                std::cout << " TransformedPoint " << TransformedPoint << std::endl;
//                // visualize pt in red
//                node->publish_3d_point(TransformedPoint[0], TransformedPoint[1], TransformedPoint[2], "map", 1, 0, 0);


                Eigen::Matrix<double, 4, 4, Eigen::RowMajor> cam_ext;
                cam_ext << 0.742273, 0.653267, 0.149242, 1.14506,
                        -0.546601, 0.719102, -0.429092, 2.65313,
                        -0.387632, 0.236927, 0.890846, -0.265419,
                        0, 0, 0, 1;
//            std::cout << " t_cam_to_map " << t_cam_to_map << std::endl;
//            cameraextrinsics.insert(std::make_pair(cam, t_map_to_cam));

                std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics_;
                cameraextrinsics_.insert(std::make_pair("kitchen", cam_ext));
                //////////  END  TESTINGGGG  //////////
                camera_extrinsics = cameraextrinsics_;
            }

            std::array<double, 4> sigma_pos = {0.3, 0.3, 0.3, 0.01};

            double sigma_landmark[3] = {0.04, 0.04, 0.04};

            // noise generation
            std::default_random_engine gen;

            std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
            std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);

            double n_x, n_y;

            // Define the bounds based on the house
            std::pair<double, double> x_bound = std::make_pair(-5, 5.0);
            std::pair<double, double> y_bound = std::make_pair(-7.0, 7.0);
//            std::pair<double, double> z_bound = std::make_pair(-1.0, 1.0);
            std::pair<double, double> z_bound = std::make_pair(0, 0);
            std::pair<double, double> theta_bound = std::make_pair(-3.1416, 3.1416);

            int num_particles =  500; // has to be multiple of 128

            double velocity = 0.01;
            double yaw_rate = 0.5;
            bool running = true;

            ParticleFilter particle_filter(num_particles);

            while (running) {
//                auto beg = std::chrono::high_resolution_clock::now();
                auto t_ = node->publish_transform(camera_extrinsics["kitchen"], "unity", "zed_cam");
                tf_broadcaster_->sendTransform(t_);
                std::vector<bool> door_status_ = {0,0,0,0,0};
                if (!particle_filter.initialized()) {
                    // Initialize the particle filter in a uniform distribution
                    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
                    node->publish_particles(particle_filter.particles);

                } else {
                    // Predict the vehicle's next state (noiseless).
                    /// not being used delta_t
//                    auto end = std::chrono::high_resolution_clock::now();
//                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
//                    double delta_t = duration.count() / 1000000.0;
                    double delta_t = 0.1; // fr debug


                    particle_filter.motion_model(delta_t, node->sigma_pos, velocity, yaw_rate, door_status_);
                    node->publish_particles(particle_filter.particles);
                }


                // get observation and skip if no observation is there
                std::vector<Observation> observations;
                Observation obs_ = node->getObservation();
                if (obs_.name != "") {
                    Eigen::Vector4d homogeneousPoint;
                    homogeneousPoint << obs_.x, obs_.y, obs_.z, 1.0;
//                    node->publish_3d_point(homogeneousPoint[0], homogeneousPoint[1], homogeneousPoint[2], "zed_cam", 1,
//                                           0, 0);
                    auto extrinsicParams = camera_extrinsics["kitchen"];

                    Eigen::Vector4d TransformedPoint;
                    TransformedPoint << extrinsicParams(0, 0) * homogeneousPoint[0] +
                                        extrinsicParams(0, 1) * homogeneousPoint[1] +
                                        extrinsicParams(0, 2) * homogeneousPoint[2] +
                                        extrinsicParams(0, 3) * homogeneousPoint[3],
                            extrinsicParams(1, 0) * homogeneousPoint[0] + extrinsicParams(1, 1) * homogeneousPoint[1] +
                            extrinsicParams(1, 2) * homogeneousPoint[2] + extrinsicParams(1, 3) * homogeneousPoint[3],
                            extrinsicParams(2, 0) * homogeneousPoint[0] + extrinsicParams(2, 1) * homogeneousPoint[1] +
                            extrinsicParams(2, 2) * homogeneousPoint[2] + extrinsicParams(2, 3) * homogeneousPoint[3],
                            extrinsicParams(3, 0) * homogeneousPoint[0] + extrinsicParams(3, 1) * homogeneousPoint[1] +
                            extrinsicParams(3, 2) * homogeneousPoint[2] + extrinsicParams(3, 3) * homogeneousPoint[3];

                    std::cout << "  camera_extrinsics[\"kitchen\"]  " << camera_extrinsics["kitchen"] << std::endl;
                    //                std::cout << " Observation ::: x " << TransformedPoint[0] << " y " << TransformedPoint[1] << " z " << TransformedPoint[2] << std::endl;
//                    node->publish_3d_point(TransformedPoint[0], TransformedPoint[1], TransformedPoint[2], "map", 0, 0,
//                                           1);


                    // observation will always be from the same camera
                    std::string cam_name = obs_.name;
                    observations.push_back(obs_);

                    // simulate the addition of noise to noiseless observation data.
                    std::vector<Observation> noisy_observations;
                    Observation obs;

                    // which is currently 1
                    for (int j = 0; j < observations.size(); ++j) {
                        n_x = N_obs_x(gen);
                        n_y = N_obs_y(gen);
                        obs = observations[j];
                        obs.x = obs.x + n_x;
                        obs.y = obs.y + n_y;
                        noisy_observations.push_back(obs);
                    }

                    // Update the weights and resample
                    particle_filter.updateWeights(sigma_landmark, noisy_observations,
                                                  camera_extrinsics[cam_name]);
                    particle_filter.resample();
                }
//                node->publish_particles(particle_filter.particles);
//
                // Calculate and output the average weighted error of the particle filter over all time steps so far.
                std::vector<Particle> particles = particle_filter.particles;
                int num_particles = particles.size();
                double highest_weight = 0.0;

                Particle best_particle;
                for (int i = 0; i < num_particles; ++i) {
                    if (particles[i].weight > highest_weight) {
                        highest_weight = particles[i].weight;
                        best_particle = particles[i];
                    }
                }

                // Fill in the message
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
                tf_broadcaster_->sendTransform(t);

                // because we want to listen to observations in this loop as well so we need to spin the node
                rclcpp::spin_some(node);
            }

        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}
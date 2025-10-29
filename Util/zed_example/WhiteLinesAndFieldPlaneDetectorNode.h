#ifndef WHITE_LINES_AND_FIELD_PLANE_DETECTOR_NODE_H
#define WHITE_LINES_AND_FIELD_PLANE_DETECTOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <mutex>
#include "WhiteLinesZedDetector.h"
#include "LineSizePixProviderZed.h"
#include <starkit_localization_msgs/msg/detected_field_marking_corner_in_self_array.hpp>
#include <starkit_localization_msgs/msg/detected_field_marking_corner_in_self.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

class WhiteLinesAndFieldPlaneDetectorNode : public rclcpp::Node {
public:
    WhiteLinesAndFieldPlaneDetectorNode();
    ~WhiteLinesAndFieldPlaneDetectorNode();

    // Get base odometry
    tf2::Transform getBaseOdom() const;

private:
    void publish_raw_image();
    void publish_processed_image();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void head_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    // ZED camera
    sl::Camera zed_;
    sl::InitParameters init_params_;
    sl::RuntimeParameters runtime_parameters_;

    // Detector objects
    std::unique_ptr<LineSizePixProviderZed> line_size_provider_;
    std::unique_ptr<WhiteLinesZedDetector> detector_;

    // ROS 2
    image_transport::Publisher white_lines_image_pub_;
    image_transport::Publisher raw_image_pub_;
    rclcpp::TimerBase::SharedPtr raw_timer_;
    rclcpp::TimerBase::SharedPtr processed_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odom_pub_;


    // Latest image and mutex
    cv::Mat latest_image_;
    std::mutex latest_image_mutex_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr raw_cb_group_;
    rclcpp::CallbackGroup::SharedPtr processed_cb_group_;

    rclcpp::Publisher<starkit_localization_msgs::msg::DetectedFieldMarkingCornerInSelfArray>::SharedPtr field_corners_pub_;

    // Head yaw
    double current_head_yaw_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr head_pose_sub_;
    geometry_msgs::msg::Pose latest_head_pose_;
    
    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // WHITE_LINES_AND_FIELD_PLANE_DETECTOR_NODE_H 
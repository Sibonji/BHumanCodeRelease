#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "WhiteLinesAndFieldPlaneDetectorNode.h"

using namespace std::chrono_literals;

tf2::Transform poseMsgToTf2(const geometry_msgs::msg::Pose& pose_msg) {
    // Extract translation
    tf2::Vector3 translation(
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z
    );

    // Extract rotation
    tf2::Quaternion rotation(
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w
    );

    // Construct tf2::Transform
    tf2::Transform tf_transform(rotation, translation);
    return tf_transform;
}

WhiteLinesAndFieldPlaneDetectorNode::WhiteLinesAndFieldPlaneDetectorNode()
: Node("white_lines_and_field_plane_detector_node")
{
    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // Publishers
    raw_image_pub_ = image_transport::create_publisher(this, "/vision/raw_image");
    white_lines_image_pub_ = image_transport::create_publisher(this, "/vision/white_lines_image");
    field_corners_pub_ = this->create_publisher<starkit_localization_msgs::msg::DetectedFieldMarkingCornerInSelfArray>("/detected_field_marking_corner_in_self", 10);
    base_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);


    // Joint state subscription
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&WhiteLinesAndFieldPlaneDetectorNode::joint_state_callback, this, std::placeholders::_1)
    );

    // Head pose subscription
    head_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/head_pose", 10,
        std::bind(&WhiteLinesAndFieldPlaneDetectorNode::head_pose_callback, this, std::placeholders::_1)
    );

    // Initialize ZED camera
    init_params_.camera_resolution = sl::RESOLUTION::HD720;
    init_params_.camera_fps = 30;    
    init_params_.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Set coordinate system as in ROS
    init_params_.coordinate_units = sl::UNIT::METER; // Set units in meters

    auto err = zed_.open(init_params_);
    if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error opening ZED camera: %s", sl::toString(err).c_str());
        throw std::runtime_error("Failed to open ZED camera");
    }

    // Manual exposure/gain test
    bool manual_exposure = false;
    if(manual_exposure) {
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);  // 0 = manual, 1 = auto
        int exposure_value = 15;
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, exposure_value);
        int gain_value = 100;
        zed_.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, gain_value);
    }

    // Enable positional tracking
    zed_.enablePositionalTracking();
    runtime_parameters_.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;

    // Create detector objects
    line_size_provider_ = std::make_unique<LineSizePixProviderZed>(zed_);
    detector_ = std::make_unique<WhiteLinesZedDetector>(*line_size_provider_);

    // Create callback groups
    raw_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    processed_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // High-frequency timer for raw image
    raw_timer_ = this->create_wall_timer(
        20ms,
        std::bind(&WhiteLinesAndFieldPlaneDetectorNode::publish_raw_image, this),
        raw_cb_group_
    );
    // Low-frequency timer for processed image
    processed_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&WhiteLinesAndFieldPlaneDetectorNode::publish_processed_image, this),
        processed_cb_group_
    );
}

WhiteLinesAndFieldPlaneDetectorNode::~WhiteLinesAndFieldPlaneDetectorNode() {
    zed_.close();
}

void WhiteLinesAndFieldPlaneDetectorNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // for (size_t i = 0; i < msg->name.size(); ++i) {
    //     if (msg->name[i] == "AAHead_yaw") {
    //         if (i < msg->position.size()) {
    //             current_head_yaw_ = msg->position[i];
    //             // std::cout << "current_head_yaw_=" << current_head_yaw_ << std::endl;
    //         }
    //         break;
    //     }
    // }
}

void WhiteLinesAndFieldPlaneDetectorNode::head_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    latest_head_pose_ = *msg;
    // Extract yaw from quaternion
    tf2::Quaternion q(
        latest_head_pose_.orientation.x,
        latest_head_pose_.orientation.y,
        latest_head_pose_.orientation.z,
        latest_head_pose_.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_head_yaw_ = yaw;

    // Publish transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "base_footprint";
    transform.child_frame_id = "head";
    
    transform.transform.translation.x = latest_head_pose_.position.x;
    transform.transform.translation.y = latest_head_pose_.position.y;
    transform.transform.translation.z = latest_head_pose_.position.z;
    
    transform.transform.rotation = latest_head_pose_.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

void WhiteLinesAndFieldPlaneDetectorNode::publish_raw_image() {
    if (zed_.grab(runtime_parameters_) == sl::ERROR_CODE::SUCCESS) {
        sl::Mat zed_image;
        zed_.retrieveImage(zed_image, sl::VIEW::LEFT);
        cv::Mat cv_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat input_frame;
        cv::cvtColor(cv_image, input_frame, cv::COLOR_BGRA2BGR);

        // Store the latest image for processing
        {
            //std::lock_guard<std::mutex> lock(latest_image_mutex_);
            latest_image_ = input_frame.clone();
        }

        // Publish the raw image
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "zed_left_camera";
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", input_frame).toImageMsg();
        raw_image_pub_.publish(msg);
    }
}

void WhiteLinesAndFieldPlaneDetectorNode::publish_processed_image() {
    cv::Mat image_to_process;
    {
        //std::lock_guard<std::mutex> lock(latest_image_mutex_);
        if (latest_image_.empty()) {
            return; // No image captured yet
        }
        image_to_process = latest_image_.clone();
    }

    tf2::Transform base_to_head_center_transform = poseMsgToTf2(latest_head_pose_);
    tf2::Transform head_center_to_zed_lens_transform;
    
    // Get the distance between the center of the camera and the left eye
    float translation_left_to_center = zed_.getCameraInformation().camera_configuration.calibration_parameters.getCameraBaseline() * 0.5f; 
    head_center_to_zed_lens_transform.setOrigin(tf2::Vector3(0, translation_left_to_center, 0));
    head_center_to_zed_lens_transform.setRotation(tf2::Quaternion(0, 0, 0, 1)); // identity rotation
    tf2::Transform base_to_zed_lens_transform = base_to_head_center_transform * head_center_to_zed_lens_transform;

    line_size_provider_->setBaseFootprintToCamera(base_to_zed_lens_transform);

    // Detect the field plane on input image and calculate related variables (line width at each pixel coords etc)
    cv::Mat line_size_frame = line_size_provider_->processFrame(image_to_process);


    detector_->setLineSizePixPlaneApproximationCoeff(
        line_size_provider_->planex_a, line_size_provider_->planex_b, line_size_provider_->planex_c, line_size_provider_->planex_d,
        line_size_provider_->planey_a, line_size_provider_->planey_b, line_size_provider_->planey_c, line_size_provider_->planey_d
    );

    // Update the curent head yaw in line detector to be able to calculate camera2self transform
    // detector_->setCurrentHeadYaw(current_head_yaw_);

    // Detect line markings using detected field plane
    std::vector<CornerResultInSelf> valid_corners;
    std::vector<LineResultInSelf> valid_lines;
    cv::Mat processed_frame = detector_->processFrame(line_size_frame, valid_corners, valid_lines);

    // Publish processed image
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "zed_left_camera";
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", processed_frame).toImageMsg();
    white_lines_image_pub_.publish(msg);

    // Publish corners as DetectedFieldMarkingCornerInSelfArray
    starkit_localization_msgs::msg::DetectedFieldMarkingCornerInSelfArray corners_msg;
    for (const auto& c : valid_corners) {
        starkit_localization_msgs::msg::DetectedFieldMarkingCornerInSelf corner_msg;
        corner_msg.pt_a.x = c.pa.x;
        corner_msg.pt_a.y = c.pa.y;
        corner_msg.pt_a.z = 0.0;
        corner_msg.pt_b.x = c.pb.x;
        corner_msg.pt_b.y = c.pb.y;
        corner_msg.pt_b.z = 0.0;
        corner_msg.pt_corner.x = c.corner.x;
        corner_msg.pt_corner.y = c.corner.y;
        corner_msg.pt_corner.z = 0.0;
        corners_msg.data.push_back(corner_msg);
    }
    field_corners_pub_->publish(corners_msg);

    // Publish odometry
    tf2::Transform base_odom = line_size_provider_->getBaseOdom();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Set pose
    odom_msg.pose.pose.position.x = base_odom.getOrigin().x();
    odom_msg.pose.pose.position.y = base_odom.getOrigin().y();
    odom_msg.pose.pose.position.z = base_odom.getOrigin().z();

    tf2::Quaternion q = base_odom.getRotation();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    base_odom_pub_->publish(odom_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<WhiteLinesAndFieldPlaneDetectorNode>();
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    rclcpp::shutdown();
    return 0;
} 
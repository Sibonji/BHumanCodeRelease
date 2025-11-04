#ifndef LINE_SIZE_PIX_PROVIDER_ZED_H
#define LINE_SIZE_PIX_PROVIDER_ZED_H

#include <tf2/LinearMath/Transform.h>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <Eigen/Dense>

class LineSizePixProviderZed {
public:
    LineSizePixProviderZed(sl::Camera& zed_camera);
    ~LineSizePixProviderZed();

    // Dummy method that takes an input image and returns it unmodified
    cv::Mat processFrame(const cv::Mat& input_frame);

    // Dummy function to fit floor plane
    bool fitFloorPlane();

    // Calculate transform that makes detected plane normal a new z, and camera central tay projection on it a new x, and a camera is floating above (0,0,0)
    void calculateRobotCentricFlatPlaneTransform(void) ;

    // Dummy function that doubles the input values
    sl::float2 getLineSizeInPixAtPix(sl::float2 pixel, float line_with_in_meters);

    // Get 3D point on plane from pixel coords
    sl::float3 planePointInWorldFromPix(sl::float2 pixel);
    Eigen::Vector3d planePointInWorldFromPix(Eigen::Vector2d pixel);


    // Get 2D pixel coords from 3D coords
    sl::float2 pixFrom3DPointInWorld(sl::float3 point);   
    // Eigen version
    Eigen::Vector2d pixFrom3DPointInWorldEigen(const Eigen::Vector3d& point);

    // Get 2D position on flattened plane from pixel coords
    sl::float2 robotCentricFlatPointOnPlaneFromPix(sl::float2 pixel);  
    cv::Point2f robotCentricFlatPointOnPlaneFromPixInMetersCv(float x, float y);
    Eigen::Vector2d robotCentricFlatPointOnPlaneFromPix(Eigen::Vector2d pixel);


    void estimateLineSizePixPlaneApproximationCoeffs(void);
    void get_plane_equation(double x1, double y1, double z1, 
                        double x2, double y2, double z2,  
                        double x3, double y3, double z3,
                        double *ra, double *rb, double *rc, double *rd);    

    sl::Pose pose; // positional tracking data
    sl::Plane plane; // detected plane 
    sl::POSITIONAL_TRACKING_STATE tracking_state = sl::POSITIONAL_TRACKING_STATE::OFF;    

    double planex_a, planex_b, planex_c, planex_d; //Plane coeffs for line width in x direction
    double planey_a, planey_b, planey_c, planey_d; //Plane coeffs for line width in y direction      

    void setBaseFootprintToCamera(const tf2::Transform& tf);

    // Get base odometry
    tf2::Transform getBaseOdom() const;

private:
    cv::Mat last_processed_frame;
    sl::Camera& zed;  // Reference to the ZED camera instance
    std::chrono::high_resolution_clock::time_point ts_last;  // Time of last plane detection
    sl::Transform resetTrackingFloorFrame; // transform between the floor plane frame and the camera frame

    sl::Transform robotCentricFlatPlaneTransform;
    Eigen::Affine3d robotCentricFlatPlaneTransformEigen;

    sl::ERROR_CODE find_plane_status = sl::ERROR_CODE::FAILURE;

    sl::float3 cam_position;
    sl::Rotation cam_rotation;
    sl::Transform cam_to_world;
    sl::Transform world_to_cam; 
    sl::CameraInformation cam_info;
    sl::CameraConfiguration cam_conf;
    sl::CalibrationParameters calib_params;
    sl::CameraParameters left_cam_params;

    sl::Transform base_footprint_to_camera_;
    Eigen::Affine3d base_footprint_to_camera_eigen_;
    double base_footprint_to_camera_yaw_;
};

#endif // LINE_SIZE_PIX_PROVIDER_ZED_H 
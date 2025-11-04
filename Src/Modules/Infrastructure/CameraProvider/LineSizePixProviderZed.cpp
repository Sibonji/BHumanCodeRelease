#include "LineSizePixProviderZed.h"
#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <Eigen/Dense>

/*
TODO: rename world to camtrack here, its better better reflects the essence of world frame used here

*/

using namespace sl;
using namespace std;

float dot(sl::float3 a, sl::float3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float norm(sl::float2 p) {
    return sqrt(pow(p.x,2) + pow(p.y,2));
}

float norm(sl::float3 p) {
    return sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
}

float norm(sl::float3 a, sl::float3 b) {
    return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}

sl::float3 normalize(sl::float3 p) {
    return p / norm(p);
}

sl::float3 transformPoint(const sl::Transform& T, const sl::float3& p) {
    return sl::float3(
        T.r00 * p.x + T.r01 * p.y + T.r02 * p.z + T.getTranslation().x,
        T.r10 * p.x + T.r11 * p.y + T.r12 * p.z + T.getTranslation().y,
        T.r20 * p.x + T.r21 * p.y + T.r22 * p.z + T.getTranslation().z
    );
}

Eigen::Affine3d slTransformToEigen(const sl::Transform& sl_transform) {
    Eigen::Matrix4d eigen_matrix;

    // Access elements using sl::Transform's (row, col) operator
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            eigen_matrix(row, col) = static_cast<double>(sl_transform(row, col));
        }
    }

    return Eigen::Affine3d(eigen_matrix);
}

Eigen::Vector3d slFloatToEigen(const sl::float3 sl_p) {
    return Eigen::Vector3d(sl_p.x, sl_p.y, sl_p.z);
}

Eigen::Affine3d tf2TransformToEigen(const tf2::Transform& tf_transform) {
    Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();

    // Extract rotation
    tf2::Quaternion tf_quat = tf_transform.getRotation();
    Eigen::Quaterniond eigen_quat(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());

    // Extract translation
    tf2::Vector3 tf_trans = tf_transform.getOrigin();
    Eigen::Vector3d eigen_trans(tf_trans.x(), tf_trans.y(), tf_trans.z());

    // Combine into affine transform
    eigen_transform.linear() = eigen_quat.toRotationMatrix();
    eigen_transform.translation() = eigen_trans;

    return eigen_transform;
}

// Function to rotate a vector around an axis by a given angle (in radians)
sl::float3 rotate_vector_around_axis(const sl::float3& vec, const sl::float3& axis, float angle_rad) {
    // Convert sl::float3 to Eigen::Vector3f
    Eigen::Vector3f v(vec.x, vec.y, vec.z);
    Eigen::Vector3f k(axis.x, axis.y, axis.z);
    k.normalize(); // Important: axis must be a unit vector

    // Rodrigues' rotation formula
    Eigen::Vector3f v_rot = v * cos(angle_rad) +
                            k.cross(v) * sin(angle_rad) +
                            k * (k.dot(v)) * (1 - cos(angle_rad));

    return sl::float3(v_rot.x(), v_rot.y(), v_rot.z());
}

LineSizePixProviderZed::LineSizePixProviderZed(sl::Camera& zed_camera) : zed(zed_camera) {
    // Initialize any necessary resources
    ts_last = chrono::high_resolution_clock::now();

    // Keep some camera related variables
    cam_info = zed.getCameraInformation();
    cam_conf = cam_info.camera_configuration;
    calib_params = cam_conf.calibration_parameters;
    left_cam_params = calib_params.left_cam;    
}

LineSizePixProviderZed::~LineSizePixProviderZed() {
    // Clean up any resources
}

cv::Mat LineSizePixProviderZed::processFrame(const cv::Mat& input_frame) {
    // Store the input frame
    last_processed_frame = input_frame.clone();
    
    fitFloorPlane();
    estimateLineSizePixPlaneApproximationCoeffs();
    
    // Return the input frame unmodified
    return last_processed_frame;
}

bool LineSizePixProviderZed::fitFloorPlane() {
    // Update pose data (used for projection of the mesh over the current image)
    tracking_state = zed.getPosition(pose);

    if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {

        // Remember some camera related variables not to compute them at each call of pix2plane etc
        // Extract transform and rotation of camera (in world frame)
        // World frame is a frame at which camera was enabled (by it's opinion)
        // Defaul ZED frame is X facing left, Y facing down, Z facing forward

        cam_position = pose.pose_data.getTranslation();
        cam_rotation = pose.pose_data.getOrientation();
        cam_to_world = pose.pose_data;
        sl::Transform world_to_cam_ = sl::Transform(cam_to_world); 
        world_to_cam_.inverse();
        world_to_cam = world_to_cam_;

        // Compute elapse time since the last call of plane detection
        auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();

        //if 500ms have spend since last request
        if(duration > 500) {
            // Update pose data (used for projection of the mesh over the current image)
            find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);
            if (find_plane_status != ERROR_CODE::SUCCESS)
                std::cout << "No plane found" << std::endl;
            ts_last = chrono::high_resolution_clock::now();
        }

        if(find_plane_status == ERROR_CODE::SUCCESS) {
            // std::cout << "Plane found OK" << std::endl;
            sl::float3 normal = plane.getNormal(); // Get the normal vector of the detected plane
            sl::float4 plane_equation = plane.getPlaneEquation(); // Get (a,b,c,d) where ax+by+cz=d
            float closest_distance = plane.getClosestDistance(pose.pose_data.getTranslation());
            // std::cout << "Camera height: " << closest_distance << " mm" << std::endl;
            // std::cout << "plane_equation=" << plane_equation << std::endl;
            // std::cout << "pose.pose_data.getTranslation()=" << pose.pose_data.getTranslation() << std::endl;

            sl::float2 size;
            size = getLineSizeInPixAtPix(sl::float2(1820/2, 720/2), 0.05);
            // std::cout << " Line size in pix:" << size << std::endl;

            // Calculate transform that makes detected plane normal a new z, and camera central tay projection on it a new x.
            calculateRobotCentricFlatPlaneTransform();

            /*
            // Test bounds
            std::vector<sl::float3> bounds = plane.getBounds();

            // Draw bonds as a solid color via OpenCV
            std::vector<cv::Point> bounds_cv;
            std::vector<std::vector<cv::Point> > bounds_cv_mult;
            for(size_t i=0; i< bounds.size(); i++) {
                sl::float2 pt0 = pixFrom3DPointInWorld(bounds[i]);
                bounds_cv.push_back(cv::Point(pt0.x, pt0.y));
            }
            bounds_cv_mult.push_back(bounds_cv);
            cv::fillPoly( last_processed_frame, bounds_cv_mult, cv::Scalar(128, 0, 0));

            // Draw bonds outer contour via OpenCV
            for(size_t i=0; i< bounds.size(); i++) {
                size_t idx1 = i+1;
                if(idx1 == bounds.size()) idx1 = 0;
                sl::float2 pt0 = pixFrom3DPointInWorld(bounds[i]);
                sl::float2 pt1 = pixFrom3DPointInWorld(bounds[idx1]);
                cv::line( last_processed_frame, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y),
                    cv::Scalar( 255, 0, 0 ),
                    2, cv::LINE_8 );                
            } 
            */           
        }
    }
    
    return true;
}

sl::float2 LineSizePixProviderZed::getLineSizeInPixAtPix(sl::float2 pixel, float line_with_in_meters) {
    
    // Just forward and backwards projection test
    // sl::float3 intersection = planePointInWorldFromPix(sl::float2(100,100));
    // sl::float2 uv = pixFrom3DPointInWorld(intersection);
    // std::cout << "Pixel coordinates: (" << uv[0] << ", " << uv[1] << ")" << std::endl;   

    sl::float3 plane_normal = plane.getNormal();

    sl::float3 plane_mid = planePointInWorldFromPix(pixel);
    sl::float3 plane_up = planePointInWorldFromPix(sl::float2(pixel.x,pixel.y-50));
    sl::float3 plane_left = planePointInWorldFromPix(sl::float2(pixel.x-50,pixel.y));
    // std::cout << "plane_mid:" << plane_mid << std::endl;
    // std::cout << "plane_up:" << plane_up << std::endl;
    // sl::float2 check = pixFrom3DPointInWorld(plane_up);
    // std::cout << "check:" << check << std::endl;

    // Generate tangent vector aligned with image vertical axis
    sl::float3 tangent_im_vert = plane_up - plane_mid;
    sl::float3 tangent_im_horz = plane_left - plane_mid;
    sl::float3 tangent_im_vert_norm = normalize(tangent_im_vert);
    sl::float3 tangent_im_horz_norm = normalize(tangent_im_horz);
    // Use Gram-Schmidt to make tangent orthogonal to normal
    // sl::float3 tangent_im_vert_norm = normalize(tangent_im_vert - dot(tangent_im_vert, plane_normal) * plane_normal);    

    // float norm_check = norm(tangent_im_vert_norm);
    // std::cout << "norm_check=" << norm_check << std::endl;
    // float dot_check = dot(tangent_im_vert, plane_normal);
    // std::cout << "dot_check=" << dot_check << std::endl;

    sl::float3 plane_up_line_width = plane_mid - tangent_im_vert_norm * line_with_in_meters;
    sl::float3 plane_left_line_width = plane_mid - tangent_im_horz_norm * line_with_in_meters;
    // std::cout << "norm_diff=" << norm(plane_mid, plane_up_line_width) << std::endl;

    sl::float2 pixel_up_line_width = pixFrom3DPointInWorld(plane_up_line_width);
    sl::float2 pixel_left_line_width = pixFrom3DPointInWorld(plane_left_line_width);
    // std::cout << "pixel_up_line_width=" << pixel_up_line_width << std::endl;

    float line_with_x = norm(pixel_left_line_width-pixel);
    float line_with_y = norm(pixel_up_line_width-pixel);

    bool verbose = false;
    if(verbose) {          
        cv::line( last_processed_frame, cv::Point(pixel.x, pixel.y), cv::Point(pixel_up_line_width.x, pixel_up_line_width.y),
                cv::Scalar( 0, 0, 255 ),
                2, cv::LINE_8 );

        cv::line( last_processed_frame, cv::Point(pixel.x, pixel.y), cv::Point(pixel_left_line_width.x, pixel_left_line_width.y),
                cv::Scalar( 0, 0, 255 ),
                2, cv::LINE_8 );                        
    }
    
    return sl::float2(line_with_x, line_with_y);
} 

void LineSizePixProviderZed::calculateRobotCentricFlatPlaneTransform(void) 
{
    Eigen::Vector3d plane_normal_world_eigen = slFloatToEigen(plane.getNormal());
    Eigen::Vector3d plane_center_world_eigen = slFloatToEigen(plane.getCenter());

    
    //Draw robot's base axes in Eigen style
    Eigen::Affine3d base_in_world_eigen = slTransformToEigen(pose.pose_data) * (base_footprint_to_camera_eigen_.inverse()); // Apply cam2base in cam local frame, so multiplying from the right
    Eigen::Vector3d base_origin_in_world_eigen = base_in_world_eigen * Eigen::Vector3d(0,0,0);
    Eigen::Vector3d base_x_axis_tip_in_world_eigen = base_in_world_eigen * Eigen::Vector3d(0.5,0,0);
    Eigen::Vector3d base_y_axis_tip_in_world_eigen = base_in_world_eigen * Eigen::Vector3d(0,0.5,0);
    Eigen::Vector3d base_z_axis_tip_in_world_eigen = base_in_world_eigen * Eigen::Vector3d(0,0,0.5); 
    
    // Project to image
    Eigen::Vector2d base_origin_in_world_pix_eigen = pixFrom3DPointInWorldEigen(base_origin_in_world_eigen);
    Eigen::Vector2d base_x_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(base_x_axis_tip_in_world_eigen);
    Eigen::Vector2d base_y_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(base_y_axis_tip_in_world_eigen);
    Eigen::Vector2d base_z_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(base_z_axis_tip_in_world_eigen);       
    
    // Draw base axes (they will hang above floor plane, also possibly rotated)
    cv::circle(last_processed_frame, cv::Point(base_origin_in_world_pix_eigen.x(), base_origin_in_world_pix_eigen.y()), 7, cv::Scalar(255,0,255), 2);
    cv::line( last_processed_frame, cv::Point(base_origin_in_world_pix_eigen.x(), base_origin_in_world_pix_eigen.y()), cv::Point(base_x_axis_in_world_pix_eigen.x(), base_x_axis_in_world_pix_eigen.y()),  cv::Scalar(0,0,255), 1); // base X axis
    cv::line( last_processed_frame, cv::Point(base_origin_in_world_pix_eigen.x(), base_origin_in_world_pix_eigen.y()), cv::Point(base_y_axis_in_world_pix_eigen.x(), base_y_axis_in_world_pix_eigen.y()),  cv::Scalar(0,255,0), 1); // base Y axis
    cv::line( last_processed_frame, cv::Point(base_origin_in_world_pix_eigen.x(), base_origin_in_world_pix_eigen.y()), cv::Point(base_z_axis_in_world_pix_eigen.x(), base_z_axis_in_world_pix_eigen.y()),  cv::Scalar(255,0,0), 1); // base Y axis    
    
   
    // Now construct a new frame in which detected white lines will be reported, called robotCentricFlatPlaneTransform
    
    // 1. Define the new Z axis (aligned with the plane normal)
    Eigen::Vector3d z_axis_eigen = plane_normal_world_eigen.normalized();  // new Z axis

    // 2. Remove component along the plane normal (z-axis) to project forward vector onto the plane to define new X axis
    Eigen::Vector3d x_hint = base_in_world_eigen.linear().col(0); // Picking X basis vector from base_in_world_eigen transform
    Eigen::Vector3d x_axis_eigen = (x_hint - x_hint.dot(z_axis_eigen) * z_axis_eigen).normalized();
    
    // 3. Compute new Y axis to complete the right-handed frame
    Eigen::Vector3d y_axis_eigen = z_axis_eigen.cross(x_axis_eigen);

    Eigen::Vector3d base2plane_vec_world_eigen = base_origin_in_world_eigen - plane_center_world_eigen;
    float dist_base_plane = base2plane_vec_world_eigen.dot(plane_normal_world_eigen);
    // std::cout << "dist_base_plane=" << dist_base_plane << std::endl;
    
    Eigen::Vector3d projected_base = base_origin_in_world_eigen - dist_base_plane * plane_normal_world_eigen;       // projection onto plane

    // Project to image
    Eigen::Vector2d projected_base_origin_in_world_pix_eigen = pixFrom3DPointInWorldEigen(projected_base);
    Eigen::Vector2d projected_base_x_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(projected_base + x_axis_eigen*0.5);
    Eigen::Vector2d projected_base_y_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(projected_base + y_axis_eigen*0.5);
    Eigen::Vector2d projected_base_z_axis_in_world_pix_eigen = pixFrom3DPointInWorldEigen(projected_base + z_axis_eigen*0.5);       
    
    // Draw projected base axes (they will be on the floor plane)
    cv::circle(last_processed_frame, cv::Point(projected_base_origin_in_world_pix_eigen.x(), projected_base_origin_in_world_pix_eigen.y()), 5, cv::Scalar(255,127,255), 3);
    cv::line( last_processed_frame, cv::Point(projected_base_origin_in_world_pix_eigen.x(), projected_base_origin_in_world_pix_eigen.y()), cv::Point(projected_base_x_axis_in_world_pix_eigen.x(), projected_base_x_axis_in_world_pix_eigen.y()),  cv::Scalar(0,0,255), 2); // base X axis
    cv::line( last_processed_frame, cv::Point(projected_base_origin_in_world_pix_eigen.x(), projected_base_origin_in_world_pix_eigen.y()), cv::Point(projected_base_y_axis_in_world_pix_eigen.x(), projected_base_y_axis_in_world_pix_eigen.y()),  cv::Scalar(0,255,0), 2); // base Y axis
    cv::line( last_processed_frame, cv::Point(projected_base_origin_in_world_pix_eigen.x(), projected_base_origin_in_world_pix_eigen.y()), cv::Point(projected_base_z_axis_in_world_pix_eigen.x(), projected_base_z_axis_in_world_pix_eigen.y()),  cv::Scalar(255,0,0), 2); // base Y axis    
    
    // Construct the robotCentricFlatPlaneTransformEigen transform
    Eigen::Affine3d world_to_flatplane_eigen = Eigen::Affine3d::Identity();
    Eigen::Matrix3d R;
    R.col(0) = x_axis_eigen;
    R.col(1) = y_axis_eigen;
    R.col(2) = z_axis_eigen;
    world_to_flatplane_eigen.linear() = R;
    world_to_flatplane_eigen.translation() = projected_base;
    
    // Assign robotCentricFlatPlaneTransformEigen transform as inverse of world_to_flatplane_eigen
    robotCentricFlatPlaneTransformEigen = world_to_flatplane_eigen.inverse(); 

}

cv::Point2f LineSizePixProviderZed::robotCentricFlatPointOnPlaneFromPixInMetersCv(float x, float y) {
    // sl::float2 res_sl = robotCentricFlatPointOnPlaneFromPix(sl::float2(x,y));
    // return cv::Point2f(res_sl.x, res_sl.y);
    Eigen::Vector2d res = robotCentricFlatPointOnPlaneFromPix(Eigen::Vector2d(x,y));
    return cv::Point2f(res.x(), res.y());
}


sl::float2 LineSizePixProviderZed::robotCentricFlatPointOnPlaneFromPix(sl::float2 pixel) 
{    
    sl::float3 pt_in_world = planePointInWorldFromPix(pixel);
    sl::float3 pt_in_flat = transformPoint(robotCentricFlatPlaneTransform, pt_in_world);
    return sl::float2(pt_in_flat.x, pt_in_flat.y);
}

Eigen::Vector2d LineSizePixProviderZed::robotCentricFlatPointOnPlaneFromPix(Eigen::Vector2d pixel) 
{    
    Eigen::Vector3d pt_in_world = planePointInWorldFromPix(pixel);
    Eigen::Vector3d pt_in_flat = robotCentricFlatPlaneTransformEigen * pt_in_world;
    return Eigen::Vector2d(pt_in_flat.x(), pt_in_flat.y());
}

// Get 2D pixel coords from 3D coords
sl::float2 LineSizePixProviderZed::pixFrom3DPointInWorld(sl::float3 point)
{

    // Backwards 3D->2D projection

    // At first transform 3d point to camera frame
    sl::float4 intersection_in_cam;
    // Stupid handmade matrix by vector multiplication, straightforward mat3f*vec3f is not working in stereolabs sdk!
    intersection_in_cam.x = world_to_cam.r00 * point.x + world_to_cam.r01 * point.y + world_to_cam.r02 * point.z + world_to_cam.getTranslation().x;
    intersection_in_cam.y = world_to_cam.r10 * point.x + world_to_cam.r11 * point.y + world_to_cam.r12 * point.z + world_to_cam.getTranslation().y;
    intersection_in_cam.z = world_to_cam.r20 * point.x + world_to_cam.r21 * point.y + world_to_cam.r22 * point.z + world_to_cam.getTranslation().z;        
    
    // Now project 3d point in camera frame to camera image plane
    float fx = left_cam_params.fx;
    float fy = left_cam_params.fy;
    float cx = left_cam_params.cx;
    float cy = left_cam_params.cy;    

    // For defatult ZED axes notation - X left, Y down, Z facing forward (thus same as camera image plane)
    // float u = (intersection_in_cam.x * fx / intersection_in_cam.z) + cx;
    // float v = (intersection_in_cam.y * fy / intersection_in_cam.z) + cy;
    
    // For ROS-style RIGHT_HANDED_Z_UP_X_FWD axes notation
    float u = (-intersection_in_cam.y * fx / intersection_in_cam.x) + cx;
    float v = (-intersection_in_cam.z * fy / intersection_in_cam.x) + cy;    

    return sl::float2(u, v);
    
}

Eigen::Vector2d LineSizePixProviderZed::pixFrom3DPointInWorldEigen(const Eigen::Vector3d& point)
{
    // Compose the world-to-cam transform in Eigen
    // world_to_cam_eigen = (slTransformToEigen(pose.pose_data)).inverse();
    Eigen::Affine3d world_to_cam_eigen = slTransformToEigen(pose.pose_data).inverse();

    // Transform the point from world to camera frame
    Eigen::Vector3d point_in_cam = world_to_cam_eigen * point;

    // Project to image using camera intrinsics
    float fx = left_cam_params.fx;
    float fy = left_cam_params.fy;
    float cx = left_cam_params.cx;
    float cy = left_cam_params.cy;

    // For ROS-style RIGHT_HANDED_Z_UP_X_FWD axes notation
    float u = (-point_in_cam.y() * fx / point_in_cam.x()) + cx;
    float v = (-point_in_cam.z() * fy / point_in_cam.x()) + cy;

    return Eigen::Vector2d(u, v);
}

void LineSizePixProviderZed::estimateLineSizePixPlaneApproximationCoeffs(void) {
    // At first we need to get proper line size in image x and y direction in any point of an image.
    // To do it quick we use plane-based interpolation estimating a plane A*x+B*y+C*z+D=0 coeffs where x and y are pisel coords, and z is interpolated line width at this point

    // To fit the line size plane, we choose 3 points in hardcoded coords
    sl::float2 ls_pim0((float)last_processed_frame.cols*0.2, (float)last_processed_frame.rows*0.8);
    sl::float2 ls_pim1((float)last_processed_frame.cols*0.8, (float)last_processed_frame.rows*0.8);
    sl::float2 ls_pim2((float)last_processed_frame.cols*0.5, (float)last_processed_frame.rows*0.4);

    // Getting line width components for this 3 points
    int px0,py0, px1,py1, px2,py2, px3,py3;
    // float dx0, dy0, dx1, dy1, dx2, dy2;

    sl::float2 size0, size1, size2;
    float line_width_in_meters = 0.05; //TODO: HARDCODE
    size0 = getLineSizeInPixAtPix(ls_pim0, line_width_in_meters);
    size1 = getLineSizeInPixAtPix(ls_pim1, line_width_in_meters);
    size2 = getLineSizeInPixAtPix(ls_pim2, line_width_in_meters);

    // Estimating fake line size plane approximation coeffs
    get_plane_equation(ls_pim0.x, ls_pim0.y, size0.x,  ls_pim1.x, ls_pim1.y, size1.x,  ls_pim2.x, ls_pim2.y, size2.x, &planex_a, &planex_b, &planex_c, &planex_d);
    get_plane_equation(ls_pim0.x, ls_pim0.y, size0.y,  ls_pim1.x, ls_pim1.y, size1.x,  ls_pim2.x, ls_pim2.y, size2.x, &planey_a, &planey_b, &planey_c, &planey_d);
 
}


// Get 3D point on plane from pixel coords
sl::float3 LineSizePixProviderZed::planePointInWorldFromPix(sl::float2 pixel)
{
    // Compute the pixel ray in camera space
    float fx = left_cam_params.fx;
    float fy = left_cam_params.fy;
    float cx = left_cam_params.cx;
    float cy = left_cam_params.cy;
    
    int u = pixel.x, v = pixel.y; // your pixel

    // Construncting ray in image plane axes notation: X facing right, Y facing down, Z facing forward
    sl::float3 ray_cam_im_plane;
    ray_cam_im_plane.x = (u - cx) / fx;
    ray_cam_im_plane.y = (v - cy) / fy;
    ray_cam_im_plane.z = 1.0f;
    ray_cam_im_plane = normalize(ray_cam_im_plane);

    // Transforming ray to ROS axes notation: X facing forward, Y facing left, Z facing up
    sl::float3 ray_cam;
    ray_cam.x =  ray_cam_im_plane.z;
    ray_cam.y = -ray_cam_im_plane.x;
    ray_cam.z = -ray_cam_im_plane.y;    

    sl::Matrix3f mat_cam = pose.getRotationMatrix();

    // Transform ray to world coordinates
    sl::float3 ray_world;
    // Stupid handmade matrix by vector multiplication, straightforward mat_cam*ray_world is not working in stereolabs sdk!
    ray_world.x = mat_cam.r00 * ray_cam.x + mat_cam.r01 * ray_cam.y + mat_cam.r02 * ray_cam.z;
    ray_world.y = mat_cam.r10 * ray_cam.x + mat_cam.r11 * ray_cam.y + mat_cam.r12 * ray_cam.z;
    ray_world.z = mat_cam.r20 * ray_cam.x + mat_cam.r21 * ray_cam.y + mat_cam.r22 * ray_cam.z;

    // Get plane equation (in world frame)
    sl::float4 plane_eq = plane.getPlaneEquation(); // [a, b, c, d]

    // Compute Ray-Plane Intersection (in world frame)
    float a = plane_eq.x;
    float b = plane_eq.y;
    float c = plane_eq.z;
    float d = plane_eq.w;

    float numerator = -(a * cam_position.x + b * cam_position.y + c * cam_position.z + d);
    float denominator = a * ray_world.x + b * ray_world.y + c * ray_world.z;

    if (fabs(denominator) > 1e-6) {
        float t = numerator / denominator;
        sl::float3 intersection = cam_position + ray_world * t;

        // std::cout << "3D point on plane (world coordinates): "
        //         << intersection.x << ", "
        //         << intersection.y << ", "
        //         << intersection.z << std::endl;

        return intersection;
    } else {
        return sl::float3(0,0,0);
    }
}

Eigen::Vector3d LineSizePixProviderZed::planePointInWorldFromPix(Eigen::Vector2d pixel)
{
    // Compute the pixel ray in camera space
    float fx = left_cam_params.fx;
    float fy = left_cam_params.fy;
    float cx = left_cam_params.cx;
    float cy = left_cam_params.cy;
    
    int u = pixel.x(), v = pixel.y(); // your pixel

    // Construncting ray in image plane axes notation: X facing right, Y facing down, Z facing forward
    Eigen::Vector3d ray_cam_im_plane;
    ray_cam_im_plane << (u - cx) / fx,
                        (v - cy) / fy,
                        1.0f;
    ray_cam_im_plane.normalize();

    // Transforming ray to ROS axes notation: X facing forward, Y facing left, Z facing up
    Eigen::Vector3d ray_cam;
    ray_cam <<  ray_cam_im_plane.z(),
               -ray_cam_im_plane.x(),
               -ray_cam_im_plane.y();

    Eigen::Matrix3d mat_cam = slTransformToEigen(pose.pose_data).linear();

    // Transform ray to world coordinates
    Eigen::Vector3d ray_world = mat_cam * ray_cam;

    // Get plane equation (in world frame)
    sl::float4 plane_eq = plane.getPlaneEquation(); // [a, b, c, d]

    // Compute Ray-Plane Intersection (in world frame)
    float a = plane_eq.x;
    float b = plane_eq.y;
    float c = plane_eq.z;
    float d = plane_eq.w;

    float numerator = -(a * cam_position.x + b * cam_position.y + c * cam_position.z + d);
    float denominator = a * ray_world.x() + b * ray_world.y() + c * ray_world.z();

    if (fabs(denominator) > 1e-6) {
        float t = numerator / denominator;
        Eigen::Vector3d intersection = slFloatToEigen(cam_position) + ray_world * t;
        return intersection;
    } else {
        return Eigen::Vector3d::Zero(3);
    }
}

void LineSizePixProviderZed::get_plane_equation(double x1, double y1, double z1, 
                        double x2, double y2, double z2,  
                        double x3, double y3, double z3,
                        double *ra, double *rb, double *rc, double *rd) 
{ 
  //taken from https://www.geeksforgeeks.org/program-to-find-equation-of-a-plane-passing-through-3-points/
    double a1 = x2 - x1; 
    double b1 = y2 - y1; 
    double c1 = z2 - z1; 
    double a2 = x3 - x1; 
    double b2 = y3 - y1; 
    double c2 = z3 - z1; 
    double a,b,c,d;
    a = b1 * c2 - b2 * c1; 
    b = a2 * c1 - a1 * c2; 
    c = a1 * b2 - b1 * a2; 
    d = (- a * x1 - b * y1 - c * z1); 
    *ra = a;
    *rb = b;
    *rc = c;
    *rd = d;
} 


void LineSizePixProviderZed::setBaseFootprintToCamera(const tf2::Transform& tf) {
    // Convert tf2::Transform to sl::Transform
    const tf2::Vector3& t = tf.getOrigin();
    const tf2::Quaternion& q = tf.getRotation();
    sl::Translation sl_t(t.x(), t.y(), t.z());
    sl::Orientation sl_q(sl::float4(q.x(), q.y(), q.z(), q.w()));
    base_footprint_to_camera_ = sl::Transform(sl_q, sl_t);

    // Extract yaw from quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    base_footprint_to_camera_yaw_ = yaw;

    //------------------------
    // Now do it in Eigen style
    base_footprint_to_camera_eigen_ = tf2TransformToEigen(tf);
}

tf2::Transform LineSizePixProviderZed::getBaseOdom() const {
    // Return base odometry as a tf2::Transform
    // This is a placeholder implementation - you may need to integrate with actual odometry data
    // Extract rotation matrix (3x3)
    sl::Transform sl_transform_cam = pose.pose_data;

    Eigen::Affine3d eig_transform_cam = slTransformToEigen(sl_transform_cam);
    Eigen::Affine3d eig_transform_base = eig_transform_cam * base_footprint_to_camera_eigen_.inverse();

    // Extract rotation and translation
    const Eigen::Matrix3d& R = eig_transform_base.rotation();
    const Eigen::Vector3d& t = eig_transform_base.translation();

    // Convert to tf2 types
    tf2::Matrix3x3 tf_rot(R(0,0), R(0,1), R(0,2),
                          R(1,0), R(1,1), R(1,2),
                          R(2,0), R(2,1), R(2,2));

    tf2::Vector3 tf_trans(t.x(), t.y(), t.z());

    // Build tf2::Transform
    tf2::Transform tf_base(tf_rot, tf_trans);

    return tf_base;
}




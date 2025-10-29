#include <opencv2/opencv.hpp>
#include <iostream>
#include <sl/Camera.hpp>
#include "WhiteLinesZedDetector.h"
#include "LineSizePixProviderZed.h"

using namespace sl;

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [--camera | <image_path>]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --camera    : Use ZED2i camera as input" << std::endl;
    std::cout << "  <image_path>: Path to input image file" << std::endl;
}

// Convert ZED Mat to OpenCV Mat
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

int main(int argc, char** argv) {
    // if (argc != 2) {
    //     printUsage(argv[0]);
    //     return -1;
    // }

    // Create instances of both detectors
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;

    // Open the camera
    auto err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error " << err << ", exit program." << std::endl;
        return -1;
    }

    // Manual exposure/gain test, works OK
    bool manual_exposure = true;
    if(manual_exposure) {
        // Disable auto exposure first
        zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);  // 0 = manual, 1 = auto

        // Set exposure value (range: 0–100 for ZED2 / ZED2i / ZED X)
        int exposure_value = 15;
        zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, exposure_value);    
        int gain_value = 100;
        zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, gain_value);
    }

    // Enable positional tracking before starting spatial mapping
    zed.enablePositionalTracking();
    
    RuntimeParameters runtime_parameters;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
    
    // the plane detection parameters can be change
    PlaneDetectionParameters plane_parameters;

    // Create an instance of LineSizePixProviderZed with the ZED camera
    LineSizePixProviderZed line_size_provider(zed);
    WhiteLinesZedDetector detector(line_size_provider);

    cv::Mat input_frame;  // Declare input_frame variable

    // Main loop
    while (true) {
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) { // Passing runtime_parameters here is crutial to get all data in world frame (!)
            // Get the left image
            sl::Mat zed_image;
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            
            // Convert to OpenCV format (already in BGR)
            cv::Mat cv_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::cvtColor(cv_image, input_frame, cv::COLOR_BGRA2BGR);

            // Process the frame with both detectors
            double start_time = static_cast<double>(cv::getTickCount());
            
            // First process with line size provider
            cv::Mat line_size_frame = line_size_provider.processFrame(input_frame);

            // Get data from line_size_provider
            detector.setLineSizePixPlaneApproximationCoeff(
                line_size_provider.planex_a, line_size_provider.planex_b, line_size_provider.planex_c, line_size_provider.planex_d,
                line_size_provider.planey_a, line_size_provider.planey_b, line_size_provider.planey_c, line_size_provider.planey_d
            );
            
            // Then process with white lines detector
            cv::Mat processed_frame = detector.processFrame(line_size_frame);

            // sl::float2 pt = line_size_provider.robotCentricFlatPointOnPlaneFromPix(sl::float2(input_frame.cols/2, input_frame.rows/2));
            // std::cout << "robotCentricFlatPointOnPlaneFromPix(im_center)=" << pt << std::endl;
            
            double end_time = static_cast<double>(cv::getTickCount());
            double processing_time = (end_time - start_time) / cv::getTickFrequency();
            std::cout << "Processing time: " << processing_time * 1000 << " ms" << std::endl;

            // Display the original and processed images
            // cv::imshow("Original Image", input_frame);
            // cv::imshow("Line Size Processed", line_size_frame);
            cv::imshow("Final Processed Image", processed_frame);

            // Check for exit key
            char key = cv::waitKey(10);
            if (key == 27) // ESC key
                break;
        }
    }

    // Close the camera
    zed.close();

    // Clean up
    cv::destroyAllWindows();
    return 0;
} 
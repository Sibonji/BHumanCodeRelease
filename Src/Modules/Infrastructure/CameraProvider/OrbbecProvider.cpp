/**
 * @file OrbbecProvider.cpp
 *
 * This file implements a module that grabs images from the Zed sensor.
 *
 * @author -
 */

 #include "OrbbecProvider.h"
 #include "Math/Range.h"
 #include "Platform/BHAssert.h"
 #include "Platform/SystemCall.h"
 #include "Platform/Thread.h"
 #include "Platform/Time.h"
 #include <iostream>
 #include <chrono>
 #include <iomanip>
 
 MAKE_MODULE(OrbbecProvider);
 
 thread_local OrbbecProvider* OrbbecProvider::theInstance = nullptr;
 
 const Rangei OrbbecProvider::settingLimits[OrbbecProvider::numOfSettings]
 {
   {0, 1},
   {-64, 64},
   {0, 100},
   {1, 10000},
   {0, 128},
   {100, 500},
   {-180, 180},
   {0, 100},
   {0, 100},
   {2800, 6500},
   {0, 1},
   {0, 1},
   {0, 32},
   {0, 3},
   {0, 1}
 };
 
 const std::unordered_map<OrbbecProvider::Setting, OrbbecProvider::Setting> OrbbecProvider::skipIfEnabled =
 {
   {OrbbecProvider::exposure, OrbbecProvider::autoExposure},
   {OrbbecProvider::gain, OrbbecProvider::autoExposure},
   {OrbbecProvider::whiteBalance, OrbbecProvider::autoWhiteBalance}
 };
 
 void OrbbecProvider::Settings::read(In& stream)
 {
   FOREACH_ENUM(Setting, setting)
     switch(setting)
     {
       case backlightCompensation:
       case autoExposure:
       case autoWhiteBalance:
       case autoExposurePriority:
       {
         bool value = static_cast<bool>((*this)[setting]);
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), value);
         (*this)[setting] = static_cast<int>(value);
         break;
       }
       case powerLineFrequency:
       {
         PowerLineFrequency value = static_cast<PowerLineFrequency>((*this)[setting]);
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), value);
         (*this)[setting] = static_cast<int>(value);
         break;
       }
       default:
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), (*this)[setting]);
     }
 }
 
 void OrbbecProvider::Settings::write(Out& stream) const
 {
   FOREACH_ENUM(Setting, setting)
     switch(setting)
     {
       case backlightCompensation:
       case autoExposure:
       case autoWhiteBalance:
       case autoExposurePriority:
       {
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), static_cast<bool>((*this)[setting]));
         break;
       }
       case powerLineFrequency:
       {
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), static_cast<PowerLineFrequency>((*this)[setting]));
         break;
       }
       default:
         Streaming::streamIt(stream, TypeRegistry::getEnumName(setting), (*this)[setting]);
     }
 }
 
 void OrbbecProvider::Settings::reg()
 {
   PUBLISH(reg);
   REG_CLASS(Settings);
   FOREACH_ENUM(Setting, setting)
     switch(setting)
     {
       case backlightCompensation:
       case autoExposure:
       case autoWhiteBalance:
       case autoExposurePriority:
         TypeRegistry::addAttribute(_type, typeid(bool).name(), TypeRegistry::getEnumName(setting));
         break;
       case powerLineFrequency:
         TypeRegistry::addAttribute(_type, typeid(PowerLineFrequency).name(), TypeRegistry::getEnumName(setting));
         break;
       default:
         TypeRegistry::addAttribute(_type, typeid(int).name(), TypeRegistry::getEnumName(setting));
     }
 }
 
 OrbbecProvider::OrbbecProvider()
   : cameraInfo(CameraInfo::upper)
 {
   theInstance = this;
   VERIFY(readCameraIntrinsics());
   VERIFY(readCameraResolution());
 
   // Initialize applied settings with a value illegal for all settings.
   std::fill(appliedSettings.begin(), appliedSettings.end(), 20000);
 
 #ifdef TARGET_BOOSTER
   // ZED initialization is performed in startStream()
 #endif
 
   setupCamera();
 }
 
 OrbbecProvider::~OrbbecProvider()
 {
   stopStream();
   theInstance = nullptr;
 }
 
 void OrbbecProvider::update(CameraImage& theCameraImage)
 {
 #ifdef TARGET_BOOSTER
   if(yuvFrameData)
   {
     const unsigned timestamp = static_cast<long long>(frameMetadataTimeOfArrival) > static_cast<long long>(Time::getSystemTimeBase())
                                ? static_cast<unsigned>(frameMetadataTimeOfArrival - Time::getSystemTimeBase()) : 100000;
     theCameraImage.setReference(cameraInfo.width / 2, cameraInfo.height, const_cast<unsigned char*>(yuvFrameData), std::max(lastImageTimestamp + 1, timestamp));
     lastImageTimestamp = theCameraImage.timestamp;
   }
   else
 #endif
   {
     theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
     theCameraImage.timestamp = Time::getCurrentSystemTime();
   }
 }
 
 void OrbbecProvider::update(JPEGImage& theJPEGImage)
 {
   theJPEGImage.fromCameraImage(theCameraImage, jpegQuality);
 }
 
 bool OrbbecProvider::readCameraIntrinsics()
 {
   InMapFile stream("cameraIntrinsics.cfg");
   bool exist = stream.exists();
   if(exist)
     stream >> cameraIntrinsics;
   return exist;
 }
 
 bool OrbbecProvider::readCameraResolution()
 {
   InMapFile stream("cameraResolution.cfg");
   bool exist = stream.exists();
   if(exist)
     stream >> cameraResolutionRequest;
   return exist;
 }
 
 bool OrbbecProvider::processResolutionRequest()
 {
   const CameraResolutionRequest::Resolutions requestedResolution = theCameraResolutionRequest.resolutions[CameraInfo::upper];
   CameraResolutionRequest::Resolutions& currentResolution = cameraResolutionRequest.resolutions[CameraInfo::upper];
   if(SystemCall::getMode() == SystemCall::Mode::physicalRobot && requestedResolution != lastResolutionRequest)
   {
     lastResolutionRequest = requestedResolution;
     switch(requestedResolution)
     {
       case CameraResolutionRequest::noRequest:
         return false;
       case CameraResolutionRequest::defaultRes:
         if(!readCameraResolution())
           currentResolution = CameraResolutionRequest::w640h480;
         return true;
       case CameraResolutionRequest::w424h240:
       case CameraResolutionRequest::w480x270:
       case CameraResolutionRequest::w640h360:
       case CameraResolutionRequest::w640h480:
       case CameraResolutionRequest::w848h480:
       case CameraResolutionRequest::w1280h720:
       case CameraResolutionRequest::w1280h800:
         currentResolution = requestedResolution;
         return true;
       default:
         FAIL("Unknown resolution.");
         return false;
     }
   }
   else
     return false;
 }
 
 void OrbbecProvider::applySettings()
 {
   FOREACH_ENUM(Setting, setting)
   {
     const auto skipCheck = skipIfEnabled.find(setting);
     if(settings[setting] != appliedSettings[setting]
        && (skipCheck == skipIfEnabled.end() || appliedSettings[skipCheck->second] != 1))
     {
       const int limitedSetting = settingLimits[setting].limit(settings[setting]);
       if(limitedSetting != settings[setting])
         OUTPUT_WARNING(TypeRegistry::getEnumName(setting) << " should be inside [" << settingLimits[setting].min
                        << ", " << settingLimits[setting].max << "], but is " << settings[setting]);
 #ifdef TARGET_BOOSTER
       // ZED camera settings would be applied here
       // Currently placeholder - ZED SDK has different API for camera controls
 #endif
       appliedSettings[setting] = settings[setting];
     }
   }
 }
 
 void OrbbecProvider::setupCamera()
 {
   // set resolution
   cameraResolutionRequest.apply(CameraInfo::upper, cameraInfo);
 
   // set opening angle
   cameraInfo.openingAngleWidth = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleWidth;
   cameraInfo.openingAngleHeight = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleHeight;
 
   // set optical center
   cameraInfo.opticalCenter.x() = cameraIntrinsics.cameras[CameraInfo::upper].opticalCenter.x() * cameraInfo.width;
   cameraInfo.opticalCenter.y() = cameraIntrinsics.cameras[CameraInfo::upper].opticalCenter.y() * cameraInfo.height;
 
   // update focal length
   cameraInfo.updateFocalLength();
 
   startStream();
 }

static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

static void writeLog(const std::string& message, const std::string& level = "INFO") {
    std::ofstream logfile("/home/booster/test/logs/zed_camera.log", std::ios_base::app);
    if (logfile.is_open()) {
        logfile << "[" << getCurrentTimestamp() << "] [" << level << "] " << message << std::endl;
    }
    // Also output to console for immediate feedback
    std::cout << "[" << getCurrentTimestamp() << "] [" << level << "] " << message << std::endl;
}
 
 void OrbbecProvider::startStream()
 {
  #ifdef TARGET_BOOSTER
  sl::InitParameters init_params;
  writeLog("Starting ZED camera initialization");

  // Set resolution based on cameraInfo
  if(cameraInfo.width == 424 && cameraInfo.height == 240) {
      init_params.camera_resolution = sl::RESOLUTION::VGA; // ZED doesn't have exact match, using closest
      writeLog("Setting resolution: 424x240 -> ZED VGA (closest match)");
  }
  else if(cameraInfo.width == 640 && cameraInfo.height == 480) {
      init_params.camera_resolution = sl::RESOLUTION::VGA;
      writeLog("Setting resolution: 640x480 -> ZED VGA");
  }
  else if(cameraInfo.width == 1280 && cameraInfo.height == 720) {
      init_params.camera_resolution = sl::RESOLUTION::HD720;
      writeLog("Setting resolution: 1280x720 -> ZED HD720");
  }
  else if(cameraInfo.width == 1920 && cameraInfo.height == 1080) {
      init_params.camera_resolution = sl::RESOLUTION::HD1080;
      writeLog("Setting resolution: 1920x1080 -> ZED HD1080");
  }
  else {
      init_params.camera_resolution = sl::RESOLUTION::AUTO;
      writeLog("Setting resolution: AUTO (no specific match found)");
  }
  
  init_params.camera_fps = 30;
  init_params.depth_mode = sl::DEPTH_MODE::NONE; // color-only pipeline
  init_params.sdk_verbose = 1; // Enable verbose mode for debugging - logs ZED SDK internal messages
  
  writeLog("ZED init parameters configured - Resolution: " + 
            std::to_string(cameraInfo.width) + "x" + std::to_string(cameraInfo.height) + 
            ", FPS: 30, Depth: NONE");
  
  const sl::ERROR_CODE open_err = zed.open(init_params);
  if(open_err != sl::ERROR_CODE::SUCCESS) {
      std::string error_msg = "ZED open error: " + std::string(sl::toString(open_err));
      std::cout << error_msg << std::endl;
      writeLog(error_msg, "ERROR");
      OUTPUT_ERROR("ZedProvider: Failed to open ZED camera - " << sl::toString(open_err));
      
      // Log specific error details for common issues
      switch(open_err) {
          case sl::ERROR_CODE::CAMERA_NOT_DETECTED:
              writeLog("No ZED camera detected. Please check USB connection.", "ERROR");
              break;
          case sl::ERROR_CODE::INVALID_RESOLUTION:
              writeLog("Invalid resolution requested for ZED camera.", "ERROR");
              break;
          case sl::ERROR_CODE::LOW_USB_BANDWIDTH:
              writeLog("Insufficient USB bandwidth. Try USB 3.0 port.", "ERROR");
              break;
          case sl::ERROR_CODE::INVALID_FIRMWARE:
              writeLog("Invalid ZED firmware. Please update ZED SDK.", "ERROR");
              break;
          default:
              writeLog("Unknown ZED camera error occurred.", "ERROR");
              break;
      }
  }
  else {
      std::cout << "ZED camera opened successfully" << std::endl;
      writeLog("ZED camera opened successfully");
      
      // Get camera information AFTER successful open
      auto camera_info = zed.getCameraInformation();
      
      writeLog("Camera model: " + std::string(sl::toString(camera_info.camera_model)));
      writeLog("Serial number: " + std::to_string(camera_info.serial_number));
      writeLog("Firmware version: " + std::to_string(camera_info.camera_configuration.firmware_version));
      writeLog("Input resolution: " + 
                std::to_string(camera_info.camera_configuration.resolution.width) + "x" + 
                std::to_string(camera_info.camera_configuration.resolution.height));
      writeLog("Input FPS: " + std::to_string(camera_info.camera_configuration.fps));
      
      // Log available resolutions for debugging
      // logAvailableResolutions();
      
      writeLog("ZED initialization completed successfully");
  }
 #endif
 }
 
  //  sl::InitParameters init_params;
   
  //  writeLog("Starting ZED camera initialization");

  //  // Set resolution based on cameraInfo
  //  if(cameraInfo.width == 424 && cameraInfo.height == 240) // NEED TO REVIEW, REALSENSE INITIALIZATION USES THIS MODE 
  //    init_params.camera_resolution = sl::RESOLUTION::VGA; // ZED doesn't have exact match, using closest
  //  else if(cameraInfo.width == 640 && cameraInfo.height == 480)
  //    init_params.camera_resolution = sl::RESOLUTION::VGA;
  //  else if(cameraInfo.width == 1280 && cameraInfo.height == 720)
  //    init_params.camera_resolution = sl::RESOLUTION::HD720;
  //  else if(cameraInfo.width == 1920 && cameraInfo.height == 1080)
  //    init_params.camera_resolution = sl::RESOLUTION::HD1080;
  //  else
  //    init_params.camera_resolution = sl::RESOLUTION::AUTO;
     
  //  init_params.camera_fps = 30;
  //  init_params.depth_mode = sl::DEPTH_MODE::NONE; // color-only pipeline
  //  init_params.sdk_verbose = 1; // Enable verbose mode for debugging // NEED TO REVIEW, READ INFO ABOUT THIS MODE TO UNDERSTAND WHAT IT DOES

  //  auto camera_info = zed.getCameraInformation();
  //  writeLog("Got camera info");
  //  writeLog("Camera model: " + std::string(sl::toString(camera_info.camera_model)));

  //  const sl::ERROR_CODE open_err = zed.open(init_params);

  //  if(open_err != sl::ERROR_CODE::SUCCESS) // NEED TO REVIEW, ADD LOGS TO SOME FILE FOR THIS CASE
  //  {
  //    std::cout << "ZED open error: " << sl::toString(open_err) << std::endl;
  //    std::string error_msg = "ZED open error: " + std::string(sl::toString(open_err));
  //    writeLog(error_msg, "ERROR");
  //    OUTPUT_ERROR("OrbbecProvider: Failed to open ZED camera - " << sl::toString(open_err));
  //  }
  // else
  // {
  //   std::cout << "ZED camera opened successfully" << std::endl;
  //   writeLog("Camera model: " + std::string(sl::toString(camera_info.camera_model)));
  //   writeLog("Serial number: " + std::to_string(camera_info.serial_number));
  //   writeLog("Firmware version: " + std::to_string(camera_info.camera_configuration.firmware_version));
  //   writeLog("ZED initialization completed successfully");
  // }
//  #endif
//  }
 
 void OrbbecProvider::stopStream()
 {
 #ifdef TARGET_BOOSTER
  writeLog("Starting ZED camera stopStream function");
  sl::InitParameters init_params;

  if(yuvFrameData)
  {
    init_params.sdk_verbose = 1; // Enable verbose mode for debugging
    yuvFrameData = nullptr;
  }
  if(zed.isOpened())
  {
    zed.close();
  }
 #endif
 }
 

//  void convertZEDImageToYUYV(const sl::Mat& zedImage, std::vector<uint8_t>& outputBuffer)
//  {
//   #ifdef TARGET_BOOSTER
//       // Convert ZED's BGRA to YUYV format
//       cv::Size sz = zedImage.size();
//       int rows = sz.height;
//       int cols = sz.width;

//       // int width = zedImage.getWidth();
//       // int height = zedImage.getHeight();

//       cv::Mat cvBGRA(rows, cols, CV_8UC4, const_cast<sl::uchar1*>(zedImage.getPtr<sl::uchar1>()));
//       // cv::Mat cvBGRA(height, width, CV_8UC4, zedImage.getPtr<sl::uchar1>(sl::MEM::CPU));

//       cv::Mat cvBGR;
//       cv::cvtColor(cvBGRA, cvBGR, cv::COLOR_BGRA2BGR);
      
//       // Convert BGR to YUV (this gives YUV 4:4:4)
//       cv::Mat yuv444;
//       cv::cvtColor(cvBGR, yuv444, cv::COLOR_BGR2YUV);
      
//       // Convert YUV 4:4:4 to YUYV 4:2:2 (pack bytes appropriately)
//       // This is a simplified version - you may need more precise conversion
//       const uint8_t* yuvData = yuv444.data;
//       uint8_t* yuyvData = outputBuffer.data();
      
//       int width = cols;
//       int height = rows;
      
//       for(int y = 0; y < height; ++y) {
//           for(int x = 0; x < width; x += 2) {
//               // Pack two pixels into YUYV format: Y0 U01 Y1 V01
//               int idx1 = (y * width + x) * 3;
//               int idx2 = (y * width + x + 1) * 3;
              
//               yuyvData[0] = yuvData[idx1];     // Y0
//               yuyvData[1] = yuvData[idx1 + 1]; // U0 (shared)
//               yuyvData[2] = yuvData[idx2];     // Y1  
//               yuyvData[3] = yuvData[idx1 + 2]; // V0 (shared)
              
//               yuyvData += 4;
//           }
//       }
//       #endif
//   }
 
//  void OrbbecProvider::waitForFrameData2()
//  {
//  #ifdef TARGET_BOOSTER
//   std::vector<uint8_t> yuvBuffer;
//   //  writeLog("Starting ZedProvider waitForFrameData2");
//    bool hasFrame = yuvFrameData != nullptr; // NEED TO REVIEW, NOT SURE THIS IS VALID WAY TO CHECK IF WE GOT NEW FRAME IN ZED
//    if(!zed.isOpened())
//      return;
     
//    sl::RuntimeParameters runtime_parameters;
//    runtime_parameters.enable_depth = false; // We only need color
   
//    if(zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
//    {
//      sl::Mat left_image;
//      zed.retrieveImage(left_image, sl::VIEW::LEFT);
     
//      frameMetadataTimeOfArrival = Time::getCurrentSystemTime();
//      if (left_image.isInit())
//      {
//       // Calculate required buffer size for YUYV format
//       // YUYV is 2 bytes per pixel (YUV 4:2:2)

//       size_t requiredSize = left_image.size().cols * left_image.size().rows * 2;

//       // Resize persistent buffer if needed
//       if(yuvBuffer.size() != requiredSize) 
//         yuvBuffer.resize(requiredSize);
      
//       // Convert ZED image to YUYV format
//         convertZEDImageToYUYV(left_image, yuvBuffer);
      
//       // Now yuvFrameData points to persistent memory
//       yuvFrameData = yuvBuffer.data();
//      }
     
//      if(!hasFrame) // NEED TO REVIEW, CHECK WHAT THIS VALUE IS AND IS IT EVEN VALID CODE, REALSENSE PROVIDER CPP, 378 LINE
//      {
//        SystemCall::say("Camera ready");
//        hasFrame = true;
//      }
//    }
//  #endif
//  }
 
void convertZEDImageToYUYV(const sl::Mat& zedImage, std::vector<uint8_t>& outputBuffer)
{
#ifdef TARGET_BOOSTER
    // Get dimensions from ZED Mat
    int width = static_cast<int>(zedImage.getWidth());
    int height = static_cast<int>(zedImage.getHeight());  
    
    // // Convert ZED's BGRA to OpenCV Mat
    // cv::Mat cvBGRA(height, width, CV_8UC4, zedImage.getPtr<sl::uchar1>(sl::MEM::CPU));
    
    // Convert ZED's BGRA to OpenCV Mat
    cv::Mat cvBGRA(static_cast<int>(height), static_cast<int>(width), CV_8UC4, zedImage.getPtr<sl::uchar1>(sl::MEM::CPU));   
    
    // Manual conversion from BGRA to YUYV
    cv::Mat bgrMat;
    cv::cvtColor(cvBGRA, bgrMat, cv::COLOR_BGRA2BGR);
    
    cv::Mat yuvMat;
    cv::cvtColor(bgrMat, yuvMat, cv::COLOR_BGR2YUV);

    // // Convert directly to YUYV using OpenCV
    // cv::Mat yuyvMat;
    // cv::cvtColor(cvBGRA, yuyvMat, cv::COLOR_BGRA2YUV_YUY2);

    // Manual conversion from YUV to YUYV
    const uint8_t* yuv = yuvMat.data;
    uint8_t* yuyv = outputBuffer.data();
    
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; x += 2) {
            size_t idx1 = (y * width + x) * 3;
            size_t idx2 = (y * width + x + 1) * 3;
            
            yuyv[0] = yuv[idx1];     // Y0
            yuyv[1] = yuv[idx1 + 1]; // U0
            yuyv[2] = yuv[idx2];     // Y1
            yuyv[3] = yuv[idx1 + 2]; // V0
            
            yuyv += 4;
        }
    }
#endif    
//     // Resize output buffer
//     size_t requiredSize = width * height * 2;
//     if (outputBuffer.size() != requiredSize) {
//         outputBuffer.resize(requiredSize);
//     }
    
//     // Copy data to output buffer
//     memcpy(outputBuffer.data(), yuyvMat.data, requiredSize);
// #endif
}

 void OrbbecProvider::waitForFrameData2()
 {
 #ifdef TARGET_BOOSTER
  std::vector<uint8_t> yuvBuffer;
  //  writeLog("Starting ZedProvider waitForFrameData2");
   bool hasFrame = yuvFrameData != nullptr; // NEED TO REVIEW, NOT SURE THIS IS VALID WAY TO CHECK IF WE GOT NEW FRAME IN ZED
   if(!zed.isOpened())
     return;
     
   sl::RuntimeParameters runtime_parameters;
   runtime_parameters.enable_depth = false; // We only need color
   
   if(zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
   {
     sl::Mat left_image;
     zed.retrieveImage(left_image, sl::VIEW::LEFT);
     
     frameMetadataTimeOfArrival = Time::getCurrentSystemTime();
     if (left_image.isInit())
     {
      // Calculate required buffer size for YUYV format
      // YUYV is 2 bytes per pixel (YUV 4:2:2)

      // size_t requiredSize = left_image.size().cols * left_image.size().rows * 2;
      size_t requiredSize = left_image.getWidth() * left_image.getHeight() * 2;

      // Resize persistent buffer if needed
      if(yuvBuffer.size() != requiredSize) {
        yuvBuffer.resize(requiredSize);
      }
      
      // Convert ZED image to YUYV format
        convertZEDImageToYUYV(left_image, yuvBuffer);
      
      // Now yuvFrameData points to persistent memory
      yuvFrameData = yuvBuffer.data();
      BH_TRACE_MSG("get yuvFrameData");
      logImgToFile(left_image);
     }
     
     if(!hasFrame) // NEED TO REVIEW, CHECK WHAT THIS VALUE IS AND IS IT EVEN VALID CODE, REALSENSE PROVIDER CPP, 378 LINE
     {
       SystemCall::say("Camera ready");
       hasFrame = true;
     }
   }
 #endif
 }

 void OrbbecProvider::logImgToFile(const sl::Mat& zedImage) {
  // Get dimensions from ZED Mat
  if (img_test_saved) 
  {
    int width = static_cast<int>(zedImage.getWidth());
    int height = static_cast<int>(zedImage.getHeight());  
    
    // // Convert ZED's BGRA to OpenCV Mat
    // cv::Mat cvBGRA(height, width, CV_8UC4, zedImage.getPtr<sl::uchar1>(sl::MEM::CPU));
    
    // Convert ZED's BGRA to OpenCV Mat
    cv::Mat cvBGRA(static_cast<int>(height), static_cast<int>(width), CV_8UC4, zedImage.getPtr<sl::uchar1>(sl::MEM::CPU));   
    
    // Manual conversion from BGRA to YUYV
    cv::Mat bgrMat;
    cv::cvtColor(cvBGRA, bgrMat, cv::COLOR_BGRA2BGR);

    // Generate timestamp for unique filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream filename;
    filename << "/home/booster/image/logs/booster_k1_capture_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3) << ".png";
    bool saveResult = cv::imwrite(filename.str(), bgrMat);
    
    img_test_saved = false;
  }

  // std::ofstream file(filename, std::ios::binary);
  // if (file.is_open()) {
  //     file.write(reinterpret_cast<const char*>(yuvBuffer.data()), yuvBuffer.size());
  //     file.close();
  //     std::cout << "Logged YUV data to: " << filename << " (" << yuvBuffer.size() << " bytes)" << std::endl;
  // } else {
  //     std::cerr << "Failed to open file for logging: " << filename << std::endl;
  // }
}
 
 bool OrbbecProvider::ok([[maybe_unused]] const bool print)
 {
   // ZED error handling would go here
   return true;
 }
 
 bool OrbbecProvider::isFrameDataComplete()
 {
 #ifdef TARGET_BOOSTER
   if(theInstance)
     return theInstance->yuvFrameData != nullptr;
   else
 #endif
     return true;
 }
 
 void OrbbecProvider::waitForFrameData()
 {
 #ifdef TARGET_ROBOT
   if(theInstance)
     theInstance->waitForFrameData2();
 #endif
 }
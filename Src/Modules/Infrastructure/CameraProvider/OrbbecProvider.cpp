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

void OrbbecProvider::startStream()
{
#ifdef TARGET_BOOSTER
  sl::InitParameters init_params;
  
  // Set resolution based on cameraInfo
  if(cameraInfo.width == 424 && cameraInfo.height == 240)
    init_params.camera_resolution = sl::RESOLUTION::VGA; // ZED doesn't have exact match, using closest
  else if(cameraInfo.width == 640 && cameraInfo.height == 480)
    init_params.camera_resolution = sl::RESOLUTION::VGA;
  else if(cameraInfo.width == 1280 && cameraInfo.height == 720)
    init_params.camera_resolution = sl::RESOLUTION::HD720;
  else if(cameraInfo.width == 1920 && cameraInfo.height == 1080)
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
  else
    init_params.camera_resolution = sl::RESOLUTION::AUTO;
    
  init_params.camera_fps = 30;
  init_params.depth_mode = sl::DEPTH_MODE::NONE; // color-only pipeline
  init_params.sdk_verbose = 1; // Enable verbose mode for debugging
  
  const sl::ERROR_CODE open_err = zed.open(init_params);
  if(open_err != sl::ERROR_CODE::SUCCESS)
  {
    std::cout << "ZED open error: " << sl::toString(open_err) << std::endl;
    OUTPUT_ERROR("OrbbecProvider: Failed to open ZED camera - " << sl::toString(open_err));
  }
  else
  {
    std::cout << "ZED camera opened successfully" << std::endl;
  }
#endif
}

void OrbbecProvider::stopStream()
{
#ifdef TARGET_BOOSTER
  if(yuvFrameData)
  {
    yuvFrameData = nullptr;
  }
  if(zed.isOpened())
  {
    zed.close();
  }
#endif
}

void OrbbecProvider::waitForFrameData2()
{
#ifdef TARGET_BOOSTER
  if(!zed.isOpened())
    return;
    
  sl::RuntimeParameters runtime_parameters;
  runtime_parameters.enable_depth = false; // We only need color
  
  if(zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
  {
    sl::Mat left_image;
    zed.retrieveImage(left_image, sl::VIEW::LEFT);
    
    frameMetadataTimeOfArrival = Time::getCurrentSystemTime();
    
    // For now, we'll set yuvFrameData to nullptr since ZED provides BGRA by default
    // In a real implementation, you'd need to convert to YUYV format
    yuvFrameData = nullptr;
    
    if(!hasFrame)
    {
      SystemCall::say("Camera ready");
      hasFrame = true;
    }
  }
#endif
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
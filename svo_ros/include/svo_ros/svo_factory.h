#pragma once

#include <memory>
#include <ros/ros.h>
#include <svo/common/camera_fwd.h>

namespace svo {

// forward declarations
class ImuHandler;
class FrameHandlerMono;
class FrameHandlerStereo;
class FrameHandlerArray;
class FrameHandlerDenseMono;

namespace factory {

/// Get IMU Handler.
std::shared_ptr<ImuHandler> getImuHandler(
    const ros::NodeHandle& pnh);

/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(
    const ros::NodeHandle& pnh,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(
    const ros::NodeHandle& pnh,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Camera-Array-SVO.
std::shared_ptr<FrameHandlerArray> makeArray(
    const ros::NodeHandle& pnh,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Camera-Array-SVO
std::shared_ptr<FrameHandlerDenseMono> makeDenseMono(
    const ros::NodeHandle& pnh);

} // namespace factory
} // namespace mono

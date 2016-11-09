// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#pragma once

#include <utility> // std::pair
#include <iostream>

#include <boost/shared_ptr.hpp>

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <svo/global.h>
#include <svo/common/types.h>

namespace svo {

// forward declarations
class FrameHandlerBase;

/// Publish visualisation messages to ROS.
class Visualizer
{
public:
  typedef std::shared_ptr<Visualizer> Ptr;
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  const std::string kWorldFrame = "world";

  ros::NodeHandle pnh_;
  size_t trace_id_ = 0;
  std::string trace_dir_;
  size_t img_pub_level_;
  size_t img_pub_nth_;
  size_t dense_pub_nth_;
  ros::Publisher pub_frames_;
  ros::Publisher pub_points_;
  ros::Publisher pub_imu_pose_;
  ros::Publisher pub_info_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_pc_;
  PointCloud::Ptr pc_;
  std::vector<ros::Publisher> pub_cam_poses_;
  std::vector<ros::Publisher> pub_dense_;
  std::vector<image_transport::Publisher> pub_images_;
  ros::Publisher pub_loop_closure_;

  tf::TransformBroadcaster br_;
  bool publish_world_in_cam_frame_;
  bool publish_map_every_frame_;
  ros::Duration publish_points_display_time_;
  bool publish_seeds_;
  bool publish_seeds_uncertainty_;
  bool trace_pointcloud_;
  double vis_scale_;
  std::ofstream ofs_states_;
  std::ofstream ofs_pointcloud_;

  Visualizer(const std::string& trace_dir,
             const ros::NodeHandle& nh_private,
             const size_t num_cameras);

  ~Visualizer() = default;

  void publishSvoInfo(
      const svo::FrameHandlerBase* const svo,
      const int64_t timestamp_nanoseconds);

  void publishImages(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  void publishImagesWithFeatures(
      const FrameBundlePtr& frame_bundle,
      const int64_t timestamp);

  void publishImuPose(
      const Transformation& T_world_imu,
      const Eigen::Matrix<double, 6, 6> Covariance,
      const int64_t timestamp_nanoseconds);

  void publishCameraPoses(
      const FrameBundlePtr& frame_bundle,
      const int64_t timestamp_nanoseconds);

  void publishBundleFeatureTracks(
      const FrameBundlePtr frames_ref,
      const FrameBundlePtr frames_cur,
      int64_t timestamp);

  void publishFeatureTracks(
      const Keypoints& px_ref,
      const Keypoints& px_cur,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const ImgPyr& img_pyr,
      const Level& level,
      const uint64_t timestamp,
      const size_t frame_index);

  void visualizeHexacopter(
      const Transformation& T_frame_world,
      const uint64_t timestamp);

  void visualizeQuadrocopter(
      const Transformation& T_frame_world,
      const uint64_t timestamp);

  void visualizeMarkers(
      const FrameBundlePtr& frame_bundle,
      const std::vector<FramePtr>& close_kfs,
      const MapPtr& map);

  void publishTrajectoryPoint(
      const Eigen::Vector3d& pos_in_vision,
      const uint64_t timestamp,
      const int id);

  void visualizeMarkersWithUncertainty(
      const FramePtr& frame,
      const std::vector<FramePtr>& close_kfs,
      const MapPtr& map,
      const float sigma_threshold);

  void publishSeedsBinary(const MapPtr& map,const float sigma_threshold);

  void publishSeeds(const MapPtr& map);

  void publishSeedsAsPointcloud(
      const Frame& frame,
      bool only_converged_seeds,
      bool reset_pc_before_publishing = true);

  void publishVelocity(
      const Eigen::Vector3d& velocity_imu,
      const uint64_t timestamp);

  void publishMapRegion(const std::vector<FramePtr>& frames);

  void publishKeyframeWithPoints(
      const FramePtr& frame,
      const uint64_t timestamp,
      const double marker_scale = 0.05);

  void publishLoopClosure(
      const FramePtr& query, const FramePtr& match,
      const Transformation& T_match_query);

  void exportToDense(const FrameBundlePtr& frame_bundle);

  void publishSeedsUncertainty(const MapPtr &map);

  void visualizeCoordinateFrames(const Transformation& T_world_cam);
};

} // end namespace svo


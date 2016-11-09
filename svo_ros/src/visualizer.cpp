// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#include <svo_ros/visualizer.h>

#include <deque>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vikit/timer.h>
#include <vikit/output_helper.h>
#include <vikit/params_helper.h>

#include <svo_msgs/DenseInput.h>
#include <svo_msgs/DenseInputWithFeatures.h>
#include <svo_msgs/Info.h>

#include <svo/frame_handler_base.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/common/seed.h>
#include <svo/tracker/feature_tracking_utils.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/map.h>
#include <svo/initialization.h>
#include <svo/img_align/sparse_img_align.h>
#include <svo/reprojector.h>

namespace svo {

Visualizer::Visualizer(
    const std::string& trace_dir,
    const ros::NodeHandle& nh_private,
    const size_t n_cameras)
  : pnh_(nh_private)
  , trace_dir_(trace_dir)
  , img_pub_level_(vk::param<int>(pnh_, "publish_img_pyr_level", 0))
  , img_pub_nth_(vk::param<int>(pnh_, "publish_every_nth_img", 1))
  , dense_pub_nth_(vk::param<int>(pnh_, "publish_every_nth_dense_input", 1))
  , pc_(new PointCloud)
  , publish_world_in_cam_frame_(vk::param<bool>(pnh_, "publish_world_in_cam_frame", true))
  , publish_map_every_frame_(vk::param<bool>(pnh_, "publish_map_every_frame", false))
  , publish_points_display_time_(vk::param<double>(pnh_, "publish_point_display_time", 0))
  , publish_seeds_(vk::param<bool>(pnh_, "publish_seeds", true))
  , publish_seeds_uncertainty_(vk::param<bool>(pnh_, "publish_seeds_uncertainty", false))
  , trace_pointcloud_(vk::param<bool>(pnh_, "trace_pointcloud", false))
  , vis_scale_(vk::param<double>(pnh_, "publish_marker_scale", 1.2))
{
  // Init ROS Marker Publishers
  pub_frames_ = pnh_.advertise<visualization_msgs::Marker>("keyframes", 10);
  pub_points_ = pnh_.advertise<visualization_msgs::Marker>("points", 10000);
  pub_imu_pose_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_imu",10);
  pub_info_ = pnh_.advertise<svo_msgs::Info>("info", 10);
  pub_markers_ = pnh_.advertise<visualization_msgs::Marker>("markers", 100);
  pub_pc_ = pnh_.advertise<PointCloud>("pointcloud", 1);
  pub_dense_.resize(n_cameras);
  pub_images_.resize(n_cameras);
  pub_cam_poses_.resize(n_cameras);
  image_transport::ImageTransport it(pnh_);
  for(size_t i=0; i<n_cameras; ++i)
  {
    pub_dense_.at(i) = pnh_.advertise<svo_msgs::DenseInputWithFeatures>("dense_input/"+std::to_string(i), 2);
    pub_images_.at(i) = it.advertise("image/"+std::to_string(i), 10);
    pub_cam_poses_.at(i) = pnh_.advertise<geometry_msgs::PoseStamped>("pose_cam/"+std::to_string(i), 10);
  }
  pub_loop_closure_ = pnh_.advertise<visualization_msgs::Marker>(
      "loop_closures", 10);

  // init tracing
  std::string states_trace_name(trace_dir_+"/trace_states.txt");
  ofs_states_.open(states_trace_name.c_str());
  ofs_states_.precision(10);
}

void Visualizer::publishSvoInfo(
    const svo::FrameHandlerBase* const svo,
    const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo);
  ++trace_id_;

  if(pub_info_.getNumSubscribers() == 0)
    return;
  VLOG(100) << "Publish SVO info";

  svo_msgs::Info msg_info;
  msg_info.header.frame_id = "/cam";
  msg_info.header.seq = trace_id_;
  msg_info.header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
  msg_info.processing_time = svo->lastProcessingTime();
  msg_info.stage = static_cast<int>(svo->stage());
  msg_info.tracking_quality = static_cast<int>(svo->trackingQuality());
  msg_info.num_matches = svo->lastNumObservations();
  pub_info_.publish(msg_info);
}

void Visualizer::publishImuPose(
    const Transformation& T_world_imu,
    const Eigen::Matrix<double, 6, 6> Covariance,
    const int64_t timestamp_nanoseconds)
{
  if(pub_imu_pose_.getNumSubscribers() == 0)
    return;
  VLOG(100) << "Publish IMU Pose";

  Eigen::Quaterniond q = T_world_imu.getRotation().toImplementation();
  Eigen::Vector3d p = T_world_imu.getPosition();
  geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(
        new geometry_msgs::PoseWithCovarianceStamped);
  msg_pose->header.seq = trace_id_;
  msg_pose->header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
  msg_pose->header.frame_id = "/imu";
  msg_pose->pose.pose.position.x = p[0];
  msg_pose->pose.pose.position.y = p[1];
  msg_pose->pose.pose.position.z = p[2];
  msg_pose->pose.pose.orientation.x = q.x();
  msg_pose->pose.pose.orientation.y = q.y();
  msg_pose->pose.pose.orientation.z = q.z();
  msg_pose->pose.pose.orientation.w = q.w();
  for(size_t i=0; i<36; ++i)
    msg_pose->pose.covariance[i] = Covariance(i%6, i/6);
  pub_imu_pose_.publish(msg_pose);
}

void Visualizer::publishCameraPoses(
    const FrameBundlePtr& frame_bundle,
    const int64_t timestamp_nanoseconds)
{
  vk::output_helper::publishTfTransform(
        frame_bundle->at(0)->T_cam_world(), ros::Time().fromNSec(timestamp_nanoseconds),
        "cam_pos", kWorldFrame, br_);

  for(size_t i = 0; i < frame_bundle->size(); ++i)
  {
    if(pub_cam_poses_.at(i).getNumSubscribers() == 0)
      return;
    VLOG(100) << "Publish camera pose " << i;

    Eigen::Quaterniond q = frame_bundle->at(i)->T_world_cam().getRotation().toImplementation();
    Eigen::Vector3d p = frame_bundle->at(i)->T_world_cam().getPosition();
    geometry_msgs::PoseStampedPtr msg_pose(new geometry_msgs::PoseStamped);
    msg_pose->header.seq = trace_id_;
    msg_pose->header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
    msg_pose->header.frame_id = "/cam"+std::to_string(i);
    msg_pose->pose.position.x = p[0];
    msg_pose->pose.position.y = p[1];
    msg_pose->pose.position.z = p[2];
    msg_pose->pose.orientation.x = q.x();
    msg_pose->pose.orientation.y = q.y();
    msg_pose->pose.orientation.z = q.z();
    msg_pose->pose.orientation.w = q.w();
    pub_cam_poses_.at(i).publish(msg_pose);
  }
}

void Visualizer::publishBundleFeatureTracks(
    const FrameBundlePtr frames_ref,
    const FrameBundlePtr frames_cur,
    int64_t timestamp)
{
  if(trace_id_ % img_pub_nth_ != 0 || !frames_ref)
    return;
  VLOG(100) << "Publish bundle feature tracks.";

  for(size_t i = 0; i < frames_ref->size(); ++i)
  {
    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    feature_tracking_utils::getFeatureMatches(
          *frames_ref->at(i), *frames_cur->at(i), &matches_ref_cur);
    publishFeatureTracks(
          frames_ref->at(i)->px_vec_, frames_cur->at(i)->px_vec_,
          matches_ref_cur, frames_cur->at(i)->img_pyr_,
          img_pub_level_, timestamp, i);
  }
}

void Visualizer::publishFeatureTracks(
    const Keypoints& px_ref,
    const Keypoints& px_cur,
    const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
    const ImgPyr& img_pyr,
    const Level& level,
    const uint64_t timestamp,
    const size_t frame_index)
{
  if(pub_images_.at(frame_index).getNumSubscribers() == 0)
    return;
  VLOG(100) << "Publish feature tracks.";
  const int scale = (1<<level);
  cv::Mat img_rgb(img_pyr[level].size(), CV_8UC3);
  cv::cvtColor(img_pyr[level], img_rgb, cv::COLOR_GRAY2RGB);
  for(size_t i = 0; i < matches_ref_cur.size(); ++i)
  {
    size_t i_ref = matches_ref_cur[i].first;
    size_t i_cur = matches_ref_cur[i].second;
    cv::line(
          img_rgb,
          cv::Point2f(px_cur(0,i_cur)/scale, px_cur(1,i_cur)/scale),
          cv::Point2f(px_ref(0,i_ref)/scale, px_ref(1,i_ref)/scale),
          cv::Scalar(0,255,0), 2);
  }
  cv_bridge::CvImage img_msg;
  img_msg.header.frame_id = "/cam";
  img_msg.header.seq = trace_id_;
  img_msg.header.stamp = ros::Time().fromNSec(timestamp);
  img_msg.image = img_rgb;
  img_msg.encoding = sensor_msgs::image_encodings::BGR8;
  pub_images_.at(frame_index).publish(img_msg.toImageMsg());
}

void Visualizer::publishImages(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  if(trace_id_ % img_pub_nth_ != 0)
    return;
  VLOG(100) << "Publish images.";

  for(size_t i = 0; i < images.size(); ++i)
  {
    if(pub_images_.at(i).getNumSubscribers() == 0)
      continue;

    // Downsample image for publishing.
    ImgPyr img_pyr;
    if (images[i].type() == CV_8UC1)
    {
      frame_utils::createImgPyramid(images[i], img_pub_level_+1, img_pyr);
    }
    else if (images[i].type() == CV_8UC3)
    {
      cv::Mat gray_image;
      cv::cvtColor(images[i], gray_image, CV_BGR2GRAY);
      frame_utils::createImgPyramid(gray_image, img_pub_level_+1, img_pyr);
    }
    else
    {
      LOG(FATAL) << "Unknown image type " << images[i].type() << "!";
    }

    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
    img_msg.header.frame_id = "/cam"+std::to_string(i);
    img_msg.image = img_pyr.at(img_pub_level_);
    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    pub_images_.at(i).publish(img_msg.toImageMsg());
  }
}

void Visualizer::publishImagesWithFeatures(
    const FrameBundlePtr& frame_bundle,
    const int64_t timestamp)
{
  if(trace_id_ % img_pub_nth_ != 0)
    return;

  for(size_t i = 0; i < frame_bundle->size(); ++i)
  {
    if(pub_images_.at(i).getNumSubscribers() == 0)
      continue;
    VLOG(100) << "Publish image with features " << i;

    FramePtr frame = frame_bundle->at(i);
    cv::Mat img_rgb;
    feature_detection_utils::drawFeatures(*frame, img_pub_level_, true, &img_rgb);
    cv_bridge::CvImage img_msg;
    img_msg.header.frame_id = "/cam";
    img_msg.header.seq = trace_id_;
    img_msg.header.stamp = ros::Time().fromNSec(timestamp);
    img_msg.image = img_rgb;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_images_.at(i).publish(img_msg.toImageMsg());
  }
}

void Visualizer::visualizeHexacopter(
      const Transformation& T_frame_world,
      const uint64_t timestamp)
{
  if(pub_frames_.getNumSubscribers() > 0)
  {
    vk::output_helper::publishCameraMarker(
        pub_frames_, "cam_pos", "cams", ros::Time().fromNSec(timestamp),
        1, 0, 0.8, Vector3d(0.,0.,1.));
  }
}

void Visualizer::visualizeQuadrocopter(
    const Transformation& T_frame_world,
    const uint64_t timestamp)
{
  vk::output_helper::publishTfTransform(
        T_frame_world, ros::Time().fromNSec(timestamp), "cam_pos", kWorldFrame, br_);

  if(pub_frames_.getNumSubscribers() > 0)
  {
    vk::output_helper::publishQuadrocopterMarkers(
          pub_frames_, "cam_pos", "cams", ros::Time().fromNSec(timestamp),
          1, 0, 0.8, Vector3d(0.,0.,1.));
  }
}


void Visualizer::visualizeMarkers(
    const FrameBundlePtr& frame_bundle,
    const std::vector<FramePtr>& close_kfs,
    const Map::Ptr& map)
{
  FramePtr frame = frame_bundle->at(0); // TODO
  visualizeHexacopter(frame->T_f_w_, ros::Time::now().toNSec());
  publishTrajectoryPoint(frame->pos(), ros::Time::now().toNSec(), trace_id_);
  if(frame->isKeyframe() || publish_map_every_frame_)
  {
    std::vector<FramePtr> frames_to_visualize = close_kfs;
    frames_to_visualize.push_back(frame);
    publishMapRegion(frames_to_visualize);
  }

  if(publish_seeds_)
    publishSeeds(map);
  if(publish_seeds_uncertainty_)
    publishSeedsUncertainty(map);
}

void Visualizer::publishTrajectoryPoint(
    const Eigen::Vector3d& pos_in_vision,
    const uint64_t timestamp,
    const int id)
{
  if(pub_points_.getNumSubscribers() > 0)
  {
    VLOG(100) << "Publish trajectory point.";
    vk::output_helper::publishPointMarker(
        pub_points_, pos_in_vision, "trajectory",
        ros::Time().fromNSec(timestamp), id, 0, 0.03*vis_scale_, Vector3d(0.,0.,0.5));
  }
}

void Visualizer::publishSeeds(const Map::Ptr& map)
{
  VLOG(100) << "Publish seeds.";
  double marker_scale = 0.03*vis_scale_;
  visualization_msgs::Marker m;
  m.header.frame_id = kWorldFrame;
  m.header.stamp = ros::Time();
  m.ns = "seeds";
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = 0; // add/modify
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.points.reserve(1000);
  for(auto kf : map->keyframes_)
  {
    const FramePtr& frame = kf.second;
    const Transformation T_w_f = frame->T_world_cam();
    for(size_t i = 0; i < frame->num_features_; ++i)
    {
      if(isSeed(frame->type_vec_[i]))
      {
        CHECK(!frame->seed_ref_vec_[i].keyframe) << "Data inconsistent";
        const Vector3d xyz = T_w_f * frame->getSeedPosInFrame(i);
        geometry_msgs::Point p;
        p.x = xyz.x();
        p.y = xyz.y();
        p.z = xyz.z();
        m.points.push_back(p);
      }
    }
  }
  pub_points_.publish(m);
}

void Visualizer::publishSeedsAsPointcloud(
    const Frame& frame,
    bool only_converged_seeds,
    bool reset_pc_before_publishing)
{
  if(pub_pc_.getNumSubscribers() == 0)
    return;
  VLOG(100) << "Publish seeds as pointcloud.";

  if(reset_pc_before_publishing)
    pc_->clear();

  pc_->header.frame_id = kWorldFrame;
  pc_->header.stamp = ros::Time::now().toNSec();
  pc_->reserve(frame.num_features_);
  for(size_t i = 0; i < frame.num_features_; ++i)
  {
    if((only_converged_seeds && isConvergedSeed(frame.type_vec_.at(i)))
       || !only_converged_seeds)
    {
      const Eigen::Vector3d xyz = frame.getSeedPosInFrame(i);
      PointType p;
      p.x = xyz.x();
      p.y = xyz.y();
      p.z = xyz.z();
      p.intensity = frame.img().at<uint8_t>(frame.px_vec_(1,i), frame.px_vec_(0,i));
      pc_->push_back(p);
    }
  }
  VLOG(30) << "Publish pointcloud of size " << pc_->size();
  pub_pc_.publish(pc_);
}

void Visualizer::publishSeedsUncertainty(const Map::Ptr& map)
{
  VLOG(100) << "Publish seed uncertainty.";
  double marker_scale = 0.01*vis_scale_;
  visualization_msgs::Marker msg_variance;
  msg_variance.header.frame_id = kWorldFrame;
  msg_variance.header.stamp = ros::Time();
  msg_variance.ns = "seeds_variance";
  msg_variance.id = 0;
  msg_variance.type = visualization_msgs::Marker::LINE_LIST;
  msg_variance.action = 0; // add/modify
  msg_variance.scale.x = marker_scale;
  msg_variance.scale.y = marker_scale;
  msg_variance.scale.z = marker_scale;
  msg_variance.color.a = 1.0;
  msg_variance.color.r = 1.0;
  msg_variance.color.g = 0.0;
  msg_variance.color.b = 0.0;
  msg_variance.points.reserve(1000);
  for(auto kf : map->keyframes_)
  {
    const FramePtr& frame = kf.second;
    const Transformation T_w_f = frame->T_world_cam();
    for(size_t i = 0; i < frame->num_features_; ++i)
    {
      if(isSeed(frame->type_vec_[i]))
      {
        CHECK(!frame->seed_ref_vec_[i].keyframe) << "Data inconsistent";

        const FloatType z_inv_max = seed::getInvMaxDepth(frame->invmu_sigma2_a_b_vec_.col(i));
        const FloatType z_inv_min = seed::getInvMinDepth(frame->invmu_sigma2_a_b_vec_.col(i));
        const Vector3d p1 = T_w_f * (frame->f_vec_.col(i) * (1.0 / z_inv_min));
        const Vector3d p2 = T_w_f * (frame->f_vec_.col(i) * (1.0 / z_inv_max));

        geometry_msgs::Point msg_point;
        msg_point.x = p1.x();
        msg_point.y = p1.y();
        msg_point.z = p1.z();
        msg_variance.points.push_back(msg_point);
        msg_point.x = p2.x();
        msg_point.y = p2.y();
        msg_point.z = p2.z();
        msg_variance.points.push_back(msg_point);
      }
    }
  }
  pub_points_.publish(msg_variance);
}

void Visualizer::publishMapRegion(const std::vector<FramePtr>& frames)
{
  VLOG(100) << "Publish map region.";
  uint64_t ts = vk::Timer::getCurrentTime();
  if(pub_pc_.getNumSubscribers() > 0)
  {
    pc_->header.frame_id = kWorldFrame;

    pcl_conversions::toPCL(ros::Time::now(), pc_->header.stamp);
    pc_->clear();
    pc_->reserve(frames.size() * 150);
    PointType p;
    for(const FramePtr& frame : frames)
    {
      for(size_t i = 0; i < frame->num_features_; ++i)
      {
        if(frame->landmark_vec_[i] == nullptr)
          continue;

        Point& point = *frame->landmark_vec_[i];
        if(point.last_published_ts_ == ts)
          continue;
        point.last_published_ts_ = ts;
        p.x = point.pos_.x();
        p.y = point.pos_.y();
        p.z = point.pos_.z();
        p.intensity = isEdgelet(frame->type_vec_[i]) ? 60 : 0;
        pc_->push_back(p);
      }
    }
    VLOG(100) << "Publish pointcloud of size " << pc_->size();
    pub_pc_.publish(pc_);
  }

  if(pub_points_.getNumSubscribers() > 0)
  {
    for(const FramePtr& frame : frames)
      publishKeyframeWithPoints(frame, ++ts);
  }
}

void Visualizer::publishKeyframeWithPoints(
    const FramePtr& frame,
    const uint64_t timestamp,
    const double marker_scale)
{
  // publish keyframe
  Transformation T_world_cam(frame->T_f_w_.inverse());
  vk::output_helper::publishFrameMarker(
      pub_frames_, T_world_cam.getRotationMatrix(),
      T_world_cam.getPosition(), "kfs", ros::Time::now(), frame->id_*10, 0, marker_scale*2.0);

  // publish point cloud and links
  Position xyz_world;
  int id = 0;
  for(size_t i = 0; i < frame->num_features_; ++i)
  {
    if(frame->landmark_vec_[i] != nullptr)
    {
      PointPtr& point = frame->landmark_vec_[i];
      if(point->last_published_ts_ == timestamp)
        continue;
      point->last_published_ts_ = timestamp;
      xyz_world = point->pos();
      id = point->id();
    }
    else if(frame->seed_ref_vec_[i].keyframe != nullptr)
    {
      const SeedRef& ref = frame->seed_ref_vec_[i];
      xyz_world = ref.keyframe->T_world_cam() * ref.keyframe->getSeedPosInFrame(ref.seed_id);
      id = -ref.keyframe->id()*1000+ref.seed_id;
    }
    else
      continue;

    if(isEdgelet(frame->type_vec_[i]))
    {
      vk::output_helper::publishPointMarker(
          pub_points_, xyz_world, "pts",
          ros::Time::now(), id, 0, marker_scale*vis_scale_,
          Vector3d(0,0.6,0), publish_points_display_time_);
    }
    else
    {
      vk::output_helper::publishPointMarker(
          pub_points_, xyz_world, "pts",
          ros::Time::now(), id, 0, marker_scale*vis_scale_,
          Vector3d(1,0,1), publish_points_display_time_);
    }

    if(trace_pointcloud_)
    {
      if(!ofs_pointcloud_.is_open())
        ofs_pointcloud_.open(trace_dir_+"/pointcloud.txt");
      ofs_pointcloud_ << xyz_world.x() << " "
                      << xyz_world.y() << " "
                      << xyz_world.z() << std::endl;
    }

    /*
    if(point->normal_set_)
    {
      vk::output_helper::publishArrowMarker(
          pub_points_, T_world_from_vision_*point->pos_,
          T_world_from_vision_.rotation_matrix()*point->normal_, 0.1,
          "normal", ros::Time::now(), point->id_, 0,  0.005,
          Vector3d(0.0, 0., 1.0));
    }
    */

  }
}

void Visualizer::publishLoopClosure(
      const FramePtr& query, const FramePtr& match,
      const Transformation& T_match_query)
{
  CHECK(query);
  const Eigen::Vector3d p_w_query = query->T_f_w_.inverse().getPosition();
  const Eigen::Vector3d p_w_match = match->T_f_w_.inverse().getPosition();
  const Eigen::Vector3d p_w_query_should =
      (match->T_f_w_.inverse() * T_match_query).getPosition();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lcs";
  marker.id = query->id() * 2;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.reserve(2);
  geometry_msgs::Point point;
  point.x = static_cast<float>(p_w_match.x());
  point.y = static_cast<float>(p_w_match.y());
  point.z = static_cast<float>(p_w_match.z());
  marker.points.push_back(point);
  point.x = static_cast<float>(p_w_query_should.x());
  point.y = static_cast<float>(p_w_query_should.y());
  point.z = static_cast<float>(p_w_query_should.z());
  marker.points.push_back(point);
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  pub_loop_closure_.publish(marker);

  marker.id = query->id() * 2 + 1;
  marker.points.clear();
  point.x = static_cast<float>(p_w_query.x());
  point.y = static_cast<float>(p_w_query.y());
  point.z = static_cast<float>(p_w_query.z());
  marker.points.push_back(point);
  point.x = static_cast<float>(p_w_query_should.x());
  point.y = static_cast<float>(p_w_query_should.y());
  point.z = static_cast<float>(p_w_query_should.z());
  marker.points.push_back(point);
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  pub_loop_closure_.publish(marker);
}

void Visualizer::exportToDense(const FrameBundlePtr& frame_bundle)
{
  VLOG(100) << "Publish dense input.";
  for(size_t cam_index = 0; cam_index < frame_bundle->size(); ++cam_index)
  {
    if(dense_pub_nth_ > 0
       && trace_id_%dense_pub_nth_ == 0
       && pub_dense_.at(cam_index).getNumSubscribers() > 0)
    {
      const FramePtr& frame = frame_bundle->at(cam_index);
      svo_msgs::DenseInputWithFeatures msg;
      msg.header.stamp = ros::Time().fromNSec(frame->getTimestampNSec());
      msg.header.frame_id = "/world";
      msg.frame_id = frame->id_;

      cv_bridge::CvImage img_msg;
      img_msg.header.stamp = msg.header.stamp;
      img_msg.header.frame_id = "camera";
      if (!frame->original_color_image_.empty())
      {
        img_msg.image = frame->original_color_image_;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
      }
      else
      {
        img_msg.image = frame->img();
        img_msg.encoding = sensor_msgs::image_encodings::MONO8;
      }
      msg.image = *img_msg.toImageMsg();

      double min_z = std::numeric_limits<double>::max();
      double max_z = std::numeric_limits<double>::min();

      Position xyz_world;
      for(size_t i = 0; i < frame->num_features_; ++i)
      {
        if(frame->landmark_vec_[i] != nullptr)
        {
          xyz_world = frame->landmark_vec_[i]->pos();
        }
        else if(frame->seed_ref_vec_[i].keyframe != nullptr)
        {
          const SeedRef& ref = frame->seed_ref_vec_[i];
          xyz_world = ref.keyframe->T_world_cam() * ref.keyframe->getSeedPosInFrame(ref.seed_id);
        }
        else
          continue;

        svo_msgs::Feature feature;
        feature.x = xyz_world(0);
        feature.y = xyz_world(1);
        feature.z = xyz_world(2);
        msg.features.push_back(feature);

        Position pos_in_frame = frame->T_f_w_ * xyz_world;
        min_z = std::min(pos_in_frame[2], min_z);
        max_z = std::max(pos_in_frame[2], max_z);
      }
      msg.min_depth = (float) min_z;
      msg.max_depth = (float) max_z;

      // publish cam in world frame
      Transformation T_world_from_cam(frame->T_f_w_.inverse());
      const Eigen::Quaterniond& q = T_world_from_cam.getRotation().toImplementation();
      const Vector3d& p = T_world_from_cam.getPosition();

      msg.pose.position.x = p[0];
      msg.pose.position.y = p[1];
      msg.pose.position.z = p[2];
      msg.pose.orientation.w = q.w();
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      pub_dense_.at(cam_index).publish(msg);
    }
  }
}

void Visualizer::visualizeCoordinateFrames(const Transformation& T_world_cam)
{
  if(pub_markers_.getNumSubscribers() == 0)
    return;

  // camera frame
  vk::output_helper::publishFrameMarker(
        pub_markers_, T_world_cam.getRotationMatrix(), T_world_cam.getPosition(),
        "cam", ros::Time::now(), 0, 0, 0.2);

  // origin frame
  vk::output_helper::publishFrameMarker(
        pub_markers_, Matrix3d::Identity(), Vector3d::Zero(),
        kWorldFrame, ros::Time::now(), 0, 0, 0.2);
}

} // end namespace svo

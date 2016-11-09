#include <ros/package.h>
#include <svo/svo.h>
#include <svo/common/imu_calibration.h>
#include <svo_ros/svo_factory.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <vikit/params_helper.h>
#include <yaml-cpp/yaml.h>
#ifdef SVO_USE_VIN_BACKEND
#include <svo_gtsam/backend_optimizer.h>
#include <svo_gtsam/backend_interface.h>
#include <svo_gtsam/graph_manager.h>
#endif

namespace svo {
namespace factory {

BaseOptions loadBaseOptions(const ros::NodeHandle& pnh, bool forward_default)
{
  BaseOptions o;
  o.max_n_kfs = vk::param<int>(pnh, "max_n_kfs", 5);
  o.use_imu = vk::param<bool>(pnh, "use_imu", false);
  o.trace_dir = vk::param<std::string>(pnh, "trace_dir", ros::package::getPath("svo")+"/trace");
  o.quality_min_fts = vk::param<int>(pnh, "quality_min_fts", 50);
  o.quality_max_fts_drop = vk::param<int>(pnh, "quality_max_drop_fts", 40);
  o.relocalization_max_trials = vk::param<int>(pnh, "relocalization_max_trials", 50);
  o.poseoptim_prior_lambda = vk::param<double>(pnh, "poseoptim_prior_lambda", 0.0);
  o.poseoptim_using_unit_sphere = vk::param<bool>(pnh, "poseoptim_using_unit_sphere", false);
  o.img_align_prior_lambda_rot = vk::param<double>(pnh, "img_align_prior_lambda_rot", 0.0);
  o.img_align_prior_lambda_trans = vk::param<double>(pnh, "img_align_prior_lambda_trans", 0.0);
  o.structure_optimization_max_pts = vk::param<int>(pnh, "structure_optimization_max_pts", 20);
  o.init_map_scale = vk::param<double>(pnh, "map_scale", 1.0);
  std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
  if(vk::param<std::string>(pnh, "kfselect_criterion", default_kf_criterion) == "FORWARD")
    o.kfselect_criterion = KeyframeCriterion::FORWARD;
  else
    o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
  o.kfselect_min_dist = vk::param<double>(pnh, "kfselect_min_dist", 0.12);
  o.kfselect_numkfs_upper_thresh = vk::param<int>(pnh, "kfselect_numkfs_upper_thresh", 120);
  o.kfselect_numkfs_lower_thresh = vk::param<double>(pnh, "kfselect_numkfs_lower_thresh", 70);
  o.kfselect_min_dist_metric = vk::param<double>(pnh, "kfselect_min_dist_metric", 0.01);
  o.kfselect_min_angle = vk::param<double>(pnh, "kfselect_min_angle", 20);
  o.kfselect_min_disparity = vk::param<double>(pnh, "kfselect_min_disparity", 40);
  o.kfselect_min_num_frames_between_kfs = vk::param<int>(pnh, "kfselect_min_num_frames_between_kfs", 2);
  o.img_align_max_level = vk::param<int>(pnh, "img_align_max_level", 4);
  o.img_align_min_level = vk::param<int>(pnh, "img_align_min_level", 2);
  o.img_align_robustification = vk::param<bool>(pnh, "img_align_robustification", false);
  o.img_align_use_distortion_jacobian =
      vk::param<bool>(pnh, "img_align_use_distortion_jacobian", false);
  o.img_align_est_illumination_gain =
      vk::param<bool>(pnh, "img_align_est_illumination_gain", false);
  o.img_align_est_illumination_offset =
      vk::param<bool>(pnh, "img_align_est_illumination_offset", false);
  o.poseoptim_thresh = vk::param<double>(pnh, "poseoptim_thresh", 2.0);
  o.update_seeds_with_old_keyframes =
      vk::param<bool>(pnh, "update_seeds_with_old_keyframes", true);
  o.use_async_reprojectors = vk::param<bool>(pnh, "use_async_reprojectors", false);
  return o;
}

DetectorOptions loadDetectorOptions(const ros::NodeHandle& pnh)
{
  DetectorOptions o;
  o.cell_size = vk::param<int>(pnh, "grid_size", 35);
  o.max_level = vk::param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.threshold_primary = vk::param<int>(pnh, "detector_threshold_primary", 10);
  o.threshold_secondary = vk::param<int>(pnh, "detector_threshold_secondary", 200);
  if(vk::param<bool>(pnh, "use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

DepthFilterOptions loadDepthFilterOptions(const ros::NodeHandle& pnh)
{
  DepthFilterOptions o;
  o.max_search_level = vk::param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.use_threaded_depthfilter = vk::param<bool>(pnh, "use_threaded_depthfilter", true);
  o.seed_convergence_sigma2_thresh = vk::param<double>(pnh, "seed_convergence_sigma2_thresh", 200.0);
  o.scan_epi_unit_sphere = vk::param<bool>(pnh, "scan_epi_unit_sphere", false);
  o.affine_est_offset= vk::param<bool>(pnh, "depth_filter_affine_est_offset", true);
  o.affine_est_gain = vk::param<bool>(pnh, "depth_filter_affine_est_gain", false);
  o.max_n_seeds_per_frame =
      vk::param<int>(pnh, "max_fts", 120) * vk::param<double>(pnh, "max_seeds_ratio", 3.0);
  return o;
}

InitializationOptions loadInitializationOptions(const ros::NodeHandle& pnh)
{
  InitializationOptions o;
  o.init_min_features = vk::param<int>(pnh, "init_min_features", 100);
  o.init_min_tracked = vk::param<int>(pnh, "init_min_tracked", 80);
  o.init_min_inliers = vk::param<int>(pnh, "init_min_inliers", 70);
  o.init_min_disparity = vk::param<double>(pnh, "init_min_disparity", 40.0);
  o.init_min_features_factor = vk::param<double>(pnh, "init_min_features_factor", 2.0);
  o.reproj_error_thresh = vk::param<double>(pnh, "reproj_err_thresh", 2.0);
  o.init_disparity_pivot_ratio = vk::param<double>(pnh, "init_disparity_pivot_ratio", 0.5);
  std::string init_method = vk::param<std::string>(pnh, "init_method", "FivePoint");
  if(init_method == "Homography")
    o.init_type = InitializerType::kHomography;
  else if(init_method == "TwoPoint")
    o.init_type = InitializerType::kTwoPoint;
  else if(init_method == "FivePoint")
    o.init_type = InitializerType::kFivePoint;
  else if(init_method == "OneShot")
    o.init_type = InitializerType::kOneShot;
  else
    SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);
  return o;
}

FeatureTrackerOptions loadTrackerOptions(const ros::NodeHandle& pnh)
{
  FeatureTrackerOptions o;
  o.klt_max_level = vk::param<int>(pnh, "klt_max_level", 4);
  o.klt_min_level = vk::param<int>(pnh, "klt_min_level", 0.001);
  return o;
}

ReprojectorOptions loadReprojectorOptions(const ros::NodeHandle& pnh)
{
  ReprojectorOptions o;
  o.max_n_kfs = vk::param<int>(pnh, "reprojector_max_n_kfs", 5);
  o.max_n_features_per_frame = vk::param<int>(pnh, "max_fts", 160);
  o.cell_size = vk::param<int>(pnh, "grid_size", 35);
  o.reproject_unconverged_seeds = vk::param<bool>(pnh, "reproject_unconverged_seeds", true);
  o.affine_est_offset = vk::param<bool>(pnh, "reprojector_affine_est_offset", true);
  o.affine_est_gain = vk::param<bool>(pnh, "reprojector_affine_est_gain", false);
  return o;
}

CameraBundle::Ptr loadCameraFromYaml(const ros::NodeHandle& pnh)
{
  std::string calib_file = vk::param<std::string>(pnh, "calib_file", "~/cam.yaml");
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for(const auto& cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}

StereoTriangulationOptions loadStereoOptions(const ros::NodeHandle& pnh)
{
  StereoTriangulationOptions o;
  o.triangulate_n_features = vk::param<int>(pnh, "max_fts", 120);
  o.max_depth_inv = vk::param<double>(pnh, "max_depth_inv", 1.0/50.0);
  o.min_depth_inv = vk::param<double>(pnh, "min_depth_inv", 1.0/0.5);
  o.mean_depth_inv = vk::param<double>(pnh, "mean_depth_inv", 1.0/2.0);
  return o;
}

ImuHandler::Ptr getImuHandler(const ros::NodeHandle& pnh)
{
  std::string calib_file = vk::param<std::string>(pnh, "calib_file", "");
  ImuCalibration imu_calib = ImuHandler::loadCalibrationFromFile(calib_file);
  imu_calib.print("Loaded IMU Calibration");
  ImuInitialization imu_init = ImuHandler::loadInitializationFromFile(calib_file);
  imu_init.print("Loaded IMU Initialization");
  ImuHandler::Ptr imu_handler(new ImuHandler(imu_calib, imu_init));
  return imu_handler;
}

void setInitialPose(const ros::NodeHandle& pnh, FrameHandlerBase& vo)
{
  Transformation T_world_imuinit(
        Quaternion(vk::param<double>(pnh, "T_world_imuinit/qw", 1.0),
                   vk::param<double>(pnh, "T_world_imuinit/qx", 0.0),
                   vk::param<double>(pnh, "T_world_imuinit/qy", 0.0),
                   vk::param<double>(pnh, "T_world_imuinit/qz", 0.0)),
        Vector3d(vk::param<double>(pnh, "T_world_imuinit/tx", 0.0),
                 vk::param<double>(pnh, "T_world_imuinit/ty", 0.0),
                 vk::param<double>(pnh, "T_world_imuinit/tz", 0.0)));
  vo.setInitialImuPose(T_world_imuinit);
}


FrameHandlerMono::Ptr makeMono(const ros::NodeHandle& pnh, const CameraBundlePtr& cam)
{
  // Create camera
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);

  // Init VO
  FrameHandlerMono::Ptr vo =
      std::make_shared<FrameHandlerMono>(
        loadBaseOptions(pnh, false),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        loadInitializationOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

FrameHandlerStereo::Ptr makeStereo(const ros::NodeHandle& pnh, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(pnh);
  init_options.init_type = InitializerType::kStereo;
  FrameHandlerStereo::Ptr vo =
      std::make_shared<FrameHandlerStereo>(
        loadBaseOptions(pnh, true),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        init_options,
        loadStereoOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

FrameHandlerArray::Ptr makeArray(const ros::NodeHandle& pnh, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(pnh);
  init_options.init_type = InitializerType::kArrayGeometric;
  init_options.init_min_disparity = 25;
  DepthFilterOptions depth_filter_options = loadDepthFilterOptions(pnh);
  depth_filter_options.verbose = true;
  FrameHandlerArray::Ptr vo =
      std::make_shared<FrameHandlerArray>(
        loadBaseOptions(pnh, true),
        depth_filter_options,
        loadDetectorOptions(pnh),
        init_options,
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

} // namespace factory
} // namespace svo

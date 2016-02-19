#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <svo/common/logging.h>
#include <svo_ros/svo_interface.h>
#include <vikit/params_helper.h>

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  {
    svo::PipelineType type = svo::PipelineType::kMono;
    if(vk::param<bool>(pnh, "pipeline_is_stereo", false))
      type = svo::PipelineType::kStereo;

    svo::SvoInterface svo(type, nh, pnh);
    if(svo.imu_handler_)
      svo.subscribeImu();
    svo.subscribeImage();
    svo.subscribeRemoteKey();

    ros::spin();
    SVO_INFO_STREAM("SVO quit");
    svo.quit_ = true;
  }
  SVO_INFO_STREAM("SVO terminated.\n");
  return 0;
}

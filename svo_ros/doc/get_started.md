### Run example bags
First source the setup file of the overlay workspace:

    source ~/svo_install_overlay_ws/devel/setup.bash

Download the test bag file from http://rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag. Launch svo node:

    roslaunch svo_ros run_from_topic.launch cam_name:=svo_test_pinhole

This will also start RVIZ. Then play the bag file:

    rosbag play airground_rig_s3_2013-03-18_21-38-48.bag

Now you should be able to observe the camera motion and sparse map in RVIZ.

You can also download another bag recorded with a fisheye camera: http://rpg.ifi.uzh.ch/datasets/test_fisheye.bag. Then you need to change the following line in `run_from_topic.launch`:

    <rosparam file="$(find svo_ros)/param/pinhole.yaml" />

to

    <rosparam file="$(find svo_ros)/param/fisheye.yaml" />
And launch svo via:

    roslaunch svo_ros run_from_topic.launch cam_name:=bluefox_25000826_fisheye

### Customize launch files
We provide several example launch files under `svo_ros/launch`:
* `run_from_topic.launch`: run svo on an topic that publishes images
* `live_nodelet.launch`/`live_nodelet_fisheye.launch`: run svo as nodelet. These files can not be launched directly, since they depend on some private packages. But they can be used as an example for using svo with other nodes.

In the launch file, you have to specify the following for svo node/nodelet, as in `run_from_topic.launch`:

    <!-- Camera topic to subscribe to -->
    <param name="cam0_topic" value="camera/image_raw" type="str" />

    <!-- Camera calibration file -->
    <param name="calib_file" value="$(arg calib_file)" />

    <!--Parameters-->
    <rosparam file="$(find svo_ros)/param/pinhole.yaml" />

  Note that SVO also supports stereo cameras.

#### Parameter files
We provide two example parameter files under `svo_ros/param`:
* `pinhole.yaml`: for relatively small field of view cameras
* `fisheye.yaml`: for cameras with wide angle lens (e.g., fisheye and catadioptric)

The parameters in these files are typical values. If you wish to change the parameters, please refer to the comments in these two files. For a example of using `SVO` with a stereo-IMU setup and a different resolution, please refer to `advanced.md`.

#### Camera calibration files
If you want to use your own camera, make sure a global shutter camera is used. A good choice is [the Bluefox camera](https://www.matrix-vision.com/USB2.0-single-board-camera-mvbluefox-mlc.html) from MatrixVision.
You can put camera calibration files under `svo_ros/calib` and load them as in `run_from_topic.launch`. We use yaml files to specify camera parameters. Please refer to `camera_calibration.md` for more details.

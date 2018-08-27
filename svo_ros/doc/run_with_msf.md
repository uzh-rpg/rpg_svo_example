### Running SVO + MSF

First, fetch and build the code for MSF in the same workspace as this repository:
```
git clone git@github.com:ethz-asl/ethzasl_msf.git
catkin build
```

When running monocular SVO with MSF for loosely-coupled VIO, we send scaled poses from SVO to the EKF.  The scale for the poses comes from a user-supplied map scale (mean scene depth), so that MSF's initial guess for the filter's scale estimate is 1.0.  Since the scale can diverge easily if the poses are not close enough to the correct scale, this initialization procedure is typically done interactively.  For example, SVO is initialized such that the features are at an expected depth (e.g. at a fixed distance from a plane) and then the filter initialization is triggered once there is a good pose and depth estimate from SVO.  This can be approximately replicated when running on datasets.

First, launch the svo, msf, and gui nodes:
```
roslaunch svo_ros svo_msf_mono.launch
```
and in a separate terminal, start the rosbag for the dataset.

In the *Quadrotor Gui* window, set the namespace to "/" and press _Connect_.
Once some frames from the dataset have been published, you should see values appear in the *SVO* fields of the GUI.  
In order to initialize both SVO and MSF, press the _Start Vision Pipeline_ button while the dataset is playing.
The motion of the camera should have as little rotation as possible after pressing the button, so try to time it so that the camera is just translating.
Once SVO is initialized, it will automatically send poses to MSF, but the scale of the pose needs to be relatively close to its true value, otherwise the scale estimate in MSF will diverge.  
If MSF initializes successfully, you should see *OK* in the _Vision Pipeline_ field in the GUI.  
After a few seconds, though, it may diverge, in which case _Vision_Pipeline_ will say *Error*, and a message like
```
[ERROR] [1535379995.319445286]: [/sensor_fusion_util] msf scale estimate diverged (scale estimate: 1.509)
```
will print to the console.
The solution to this is to attempt initialization a few times using the _Start Vision Pipeline_ button, and to adjust the *map_scale* parameter in `param/euroc_mono_imu.yaml` so that it more closely approximates the true scale of the features that SVO has initialized.

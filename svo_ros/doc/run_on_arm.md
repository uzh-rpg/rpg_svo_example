# Running on ARM
The ARM binaries do not include the GUI for SVO status (`rqt_svo`), since we usually run the applications on an ARM processor in a headless mode (e.g., via ssh). Therefore, you need to run the GUI (i.e., RVIZ and `rqt_svo` window) on another PC. In order to use the `rqt_svo` window, you need to have `SVO` set up on the PC as well.

After installing `SVO` on both ARM and PC, you need to make the programs on the ARM and the PC share the same ROS master. Suppose the ARM and the PC are in the same network, first change the `ROS_MASTER_URI` to the IP address of the PC, where we will run the `roscore`, in `scripts/setupros_ip_arm.sh`:
```
# On the ARM, in setupros_ip_arm.sh
...
# change the ip to your PC where you run the roscore
export ROS_MASTER_URI=http://192.168.200.7:11311
...
```
Then run the following scripts (under `scripts` folder) respectively:
```
# On the PC
source setupros_ip_pc.sh
# On the ARM:
source setupros_ip_arm.sh
```
You may want to put the above lines in your `.bashrc`, otherwise you have to do it for **every new terminal** you open.

Then run `roscore` on the PC:
```
# On the PC
roscore
```
You should double check that the output of `roscore` reflects that the master is on the IP you set.

Afte setting up the `roscore`, run the following on the PC
```
# On the PC: you also need to manually load the rviz configuration `rviz_config.rviz`
rosrun rviz rviz
# After sourcing your SVO installation on the PC
rqt_svo
```
and on the ARM:
```
# On the ARM
source ~/svo_install_overlay_ws/devel/setup.bash
roslaunch svo_ros run_from_topic_arm.launch cam_name:=svo_test_pinhole
```
Download the test bag from [here](http://rpg.ifi.uzh.ch/svo2/svo_test_short.bag), and run in another terminal on the ARM:
```
# On the ARM
# Yes, you can play the bag on the PC but there will be some delay depending on your network status
rosbag play svo_test_short.bag
```
Now you should see something similar to the output in `install.md`.  Note that if you have different version of ROS on the PC and ARM, some messages will not appear properly.

If you do not care about visualization, you can just install `SVO` on the ARM, launch the node and play the bag and ignore the settings on PC and the ROS master. The topics containing the output of `SVO` are published anyway.

## Parameters for ARM
Now we have a look at the what settings you need to use to make `SVO` run on an ARM processor. The example launch file is  `launch/run_from_topic_arm.launch`.

Because the ARM processor has limited computational power, and publishing images requires quite some computation for serialization, we set the following:
```
<param name="publish_img_pyr_level" value="2" />
<param name="publish_every_nth_img" value="10" />
```
which means we publish once every 10 images and publish the down-sampled one.

We also provide a more lightweight setting in `param/fast_pinhole.yaml` to further reduce the required computation (e.g., track less features). It is fine to use the normal setup `param/pinhole.yaml` if you only run `SVO`. But if you have other applications to run on the ARM processor, it is usually needed to tune the parameters to fit the computational budget.


## Nodelet
As mentioned above, image serialization is expensive. A common use case for an ARM processor is to use the camera attached to the embedded system and do computation on-board. In this situation, we can use [nodelet](http://wiki.ros.org/nodelet) provided by ROS to reducing the overhead of serializing/deserializing the images.

The binaries allow to use `SVO` in a nodelet form, and an example can be found in  `launch/run_from_topic_arm.launch`. You will need to run your camera driver in the same nodelet. For porting nodes to nodelets and running nodelets, please refer to [the official tutorial](http://wiki.ros.org/nodelet/Tutorials).

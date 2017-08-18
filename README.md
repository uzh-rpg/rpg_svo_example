This repository contains examples to use `SVO 2.0` binaries in various configurations:
* monocular
* stereo
* stereo/monocular + IMU
* pinhole/fisheye/catadioptric cameras are supported

For detailed instructions, please refer to the documentation under `rpg_svo_example/svo_ros/doc`.
We provide several launch files under `rpg_svo_example/svo_ros/launch` for different datasets, including the [EuRoC](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) datasets.
These can be used as a reference for customization according to specific needs.

To use `SVO 2.0` as a binary form, you need to request the binaries of `SVO 2.0` from [here](http://rpg.ifi.uzh.ch/svo2.html).
The binaries you get already include a copy of this repository, therefore they are self-contained.

This repository will also be used for the following purposes:
* Keep track of the update of the binaries
* Add new launch files and example code to use the binaries

If the binaries are updated, you can download the new version from the original links in the email you received.

**If you have any problem regarding using `SVO 2.0` binaries,
you can either start an issue or contact Zichao Zhang (zzhang AT ifi DOT uzh DOT ch).**

# Binaries changelog
**17.07.2017** Initial Release

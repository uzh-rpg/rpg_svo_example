### Prerequisites
#### System
Make sure your system meets the following requirements:
* Ubuntu 14.04 64 bit
* gcc version 4.8
* ROS version `indigo` ([installation guide](http://wiki.ros.org/indigo/Installation/Ubuntu)).

For 16.04, install the ROS version `kinetic` instead ([installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)) and the default gcc version of Ubuntu 16.04 should work.

#### Install catkin tools
We use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) to build workspace. It is recommended to use catkin 0.3.1:

    sudo apt-get install python-pip
    sudo pip install catkin-tools==0.3.1

Remember to remove the previous version that you are using.


### Install

#### Create the install workspace
Download the install workspace from: http://rpg.ifi.uzh.ch/software/svo_install_ws_1404.zip.
Extract the zip file to where you want to install the binaries (e.g., your home folder).
Then we should have a folder `~/svo_install_ws` with a subfolder named `install`.

Run the script within the workspace to fix some hardcoded paths:

    cd svo_install_ws
    ./fix_path.sh

#### Create an overlay workspace
Now we will create a workspace to use the binaries we just downloaded. Before proceeding, make sure you have already source the setup file from ROS (`/opt/ros/indigo/setup.bash`).

First source the install workspace:

    cd ~
    source svo_install_ws/install/setup.bash

Create a new catkin workspace:

    mkdir svo_install_overlay_ws
    cd svo_install_overlay_ws
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

Now, this workspace should overlay both the ros installation and the `svo_install_ws`. Typing `catkin config`, you should see:

    Extending:    [env] /home/deploy_user/svo_install_ws/install:/opt/ros/indigo

Clone the `rpg_svo_example` repository to use the binary:

    mkdir src
    cd src
    git clone git@github.com:uzh-rpg/rpg_svo_example.git
    catkin build
Then you can use SVO via `rpg_svo_example`.

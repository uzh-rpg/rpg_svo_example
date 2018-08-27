### Running SVO + GTSAM

When compiling this package, ensure that CMakeLists.txt will build the interface to the GTSAM backend: `set(USE_GTSAM true)`

There is a launch file for running a monocular SVO+GTSAM node with one of the EuRoC datasets:
```
roslaunch svo_ros svo_gtsam.launch
```

This code is, unfortunately, very sensitive to underconstrained variables when performing bundle adjustment.  You're likely to see errors like the following:
```
Indeterminant linear system detected while working near variable
7782220156096221807 (Symbol: l4719).

Thrown when a linear system is ill-posed.  The most common cause for this
error is having underconstrained variables.  Mathematically, the system is
underdetermined.  See the GTSAM Doxygen documentation at
http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for
more information.
Variable has type 'l' and index 4719
terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
  what():  
Indeterminant linear system detected while working near variable
7782220156096221807 (Symbol: l4719).

Thrown when a linear system is ill-posed.  The most common cause for this
error is having underconstrained variables.  Mathematically, the system is
underdetermined.  See the GTSAM Doxygen documentation at
http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for
more information.
*** Aborted at 1535377507 (unix time) try "date -d @1535377507" if you are using GNU date ***
PC: @     0x7fbc5b105428 gsignal
*** SIGABRT (@0x3e800004fe4) received by PID 20452 (TID 0x7fbc3d285700) from PID 20452; stack trace: ***
    @     0x7fbc5b1054b0 (unknown)
    @     0x7fbc5b105428 gsignal
    @     0x7fbc5b10702a abort
    @     0x7fbc5b73f84d __gnu_cxx::__verbose_terminate_handler()
    @     0x7fbc5b73d6b6 (unknown)
    @     0x7fbc5b73d701 std::terminate()
    @     0x7fbc5b768d38 (unknown)
    @     0x7fbc5a0086ba start_thread
    @     0x7fbc5b1d741d clone
[svo-2] process has died [pid 20452, exit code -6, cmd /home/jeff/workspaces/svo_msf_overlay_ws/devel/lib/svo_ros/svo_node __name:=svo __log:=/home/jeff/.ros/log/3b5659b8-a9ff-11e8-aebd-54ee752cade0/svo-2.log].
log file: /home/jeff/.ros/log/3b5659b8-a9ff-11e8-aebd-54ee752cade0/svo-2*.log
```

These failures can be expected when running on datasets, during which you have less control over initialization and camera motion.  Using this with a live camera _may_ provide better results, but this is a systematic issue with GTSAM, in that the solver is not robust to indeterminant systems, so it is the responsibility of the user to only place good measurements into the system.  Unfortunately, even with reasonable checks on the landmark measurements, these underconstrained variables still appear intermittently. You will likely need to just re-run the experiment repeatedly to try to obtain a successful trial.  

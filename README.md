<h1>EDT-Lunabotics-2026</h1>

<img src="https://i.imgur.com/KgdN7f8.png" style="width: 200px" alt="EDT-Logo"></img>
<hr>
<h2>Packages</h2>
    <ul>
        <li>neptune_bringup</li>
        <li>config_pkg</li> 
        <li>description_pkg</li>
        <li>interfaces_pkg</li>
        <li>logger_pkg</li>
        <li>msg_pkg</li>
        <li>navigation_pkg</li>
        <li>scripts</li>
        <li>sim_pkg</li>
        <li>teleop_pkg</li>
        <li>third_party_packages</li>
        <li>util_pkg</li>
        <li>controller_pkg</li>
        <li>vision_pkg</li>
        <li>webgui_pkg</li>
        <li>interfaces_pkg</li>
        <li>localization_pkg</li>
    </ul>

<h2>Description</h2>
<hr>
<p>This is a GitHub repository created for the University of Illinois Chicago Engineering Design Team
for the NASA Lunabotics 2026 competition.</p>

<h2>Potential Bugs and their fixes.</h2>
<p>Default ("Zeroed") Lift / Tilt Positions:</p>

    tilts: 5 5/8 inches

    lifts: 6 13/16 inches
    
<p>Video Cameras not showing (and not throwing the error: "Failed to open USB RGB camera 1 (/dev/video6)")" etc</p> 

    fix: run "v4l2-ctl --list-devices" in /home/Desktop/robot_WS
    find the webcameras, and put the top directory, itll look /dev/videox, where x is any number, and paste it into rs_camera_node.cpp in vision_pkg. this requires recompiling- colcon build --packages-select vision_pkg

<p>"ROS Webbridge is overloaded- restarting in 2ms"</p>

    fix: okay so we started the robot too many times on the same uptime for the jetson. The cache is overloaded- and unfourtently the only fix is to restart the Jetson entirely. This happens after starting the robot 5+ times on the same uptime.

<p>/joy topic not found</p>

    there should be /joy topic always, run ros topic list and look for /joy. then echo ros topic echo /joy
    if nothing comes up, something is wrong, likely the ROS bridge being overloaded. restart the jetson.

<p>Building and sourcing throwing an error.</p>

    fix: build and remove cached artifiacts. colcon build --cmake-clean-cache

<p>Controller not sending signals, despite /joy being up</p>

    fix: you are not pressing the trigger! press right or left trigger to ensure connection is through. check ros topic echo /joy for any changes. 

<p>CAN Interface not found / CAN is busy</p>

    fix: sudo ip link set can0 up type can bitrate 1000000 || OR || lift E-stop (if busy).
    
<p>Controller node dies as canbus is overloaded, yet e-stop is up</p>

    fix: restart the NUC, ensure that the canusb is only connected AFTER the NUC is powered on.

<p>RPLiDar Internal Error</p>

    fix: ensure lidar power cable is directly plugged into the NUC, and not into the USB extender. restart the nuc after.

<p> robot isnt moving, no heartbeat sent.</p>

    fix: sometimes when the E-stop is pushed down midway and can gets consistently full, the controller node stops sending heartbeats. please run "ros2 run controller_pkg controller_node &"

<p>Clock Skew</p>

    fix: run "date" on the jetson. you might get a date/time that is off from the current time / date. then, run "sudo date -s "2025-05-15 10:00:00" (change the date to the current date) and run colcon build.

    if you still encounter clock skew after this, you will have to "touch" all files to replace their time and date- "find . -exec touch {} +" IN THE ROOT OF THE WORKSPACE (ROBOT_WS)! !! DOING THIS OUTSIDE WILL BREAK THE JETSON!

    then you may need to clear artificatsion, in robot_WS do "rm -rf build/ install/ log/" and then finally build again, "colcon build --symlink-install"

<p>Depositing Not working / Freezes up.</p>

    the service isnt displaying. press b to kill the service then try auto depositing again

<h3>Clone this Github Repo</h3>

    git clone git@github.com/educationmoment/EDT-Lunabotics-2026.git && cd EDT-Lunabotics-2026/

<p>Setup UDEV RULES and General Installation Scripts:</p>
    
    cd src/scripts
    chmod +x setup_udev_rules.sh
    chmod +x install_dependencies.sh
    sudo ./setup_udev_rules.sh && ./install_dependencies.sh

<p>The script periodically asks for user input and was designed to run inside a Docker container.
It updates the system, installs the Robotics Operating System (ROS2 Humble), and several dependencies.</p>

<h2>Control Diagram</h2>

<img src="https://i.imgur.com/vq8L5NY.png" style="width: 200px" alt="EDT-Logo"></img>

<h2>Setup</h2>
<hr>
<h3>Build the Project</h3>
<p>From within the workspace, run the following to build the entire project.</p>
<p>Note that building RTABMAP can take awhile, so its best to select packages to build individually.</p>


    colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DWITH_OPENCV=ON -DWITH_APRILTAG=ON -DWITH_OPENGV=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --parallel-workers 1 # Modify number as needed

<p> SparkCAN and Control Area Networks (CAN) will require the setup of the CAN interface on the host system.
A CAN interface should be immediately visible upon connecting the USB-to-CAN adapter. It may initially show as <strong>DOWN</strong>.
Configure the CAN interface using</p>

    sudo ip link set can0 up type can bitrate 1000000

<p>At this point, launch the robot using</p>

    ros2 launch neptune_bringup test.launch.py

<p>The pilot can access the WebGUI by visiting</p>

    http://192.168.0.131:59440/pilot

<p>You can also view the health status of the robot with </p>

    ros2 topic echo health_topic

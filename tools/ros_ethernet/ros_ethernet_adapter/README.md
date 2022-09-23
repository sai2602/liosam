# ANavS ROS-Ethernet Adapter (REA)

*Best viewed using a markdown viewer. For example, using webbrowser [extension for Google Chrome](https://chrome.google.com/webstore/detail/markdown-reader/gpoigdifkoadgajcincpilkjmejcaanc).*

The ROS-Ethernet Adapter is a command line tool for Linux that provides an adapter between ANavS TCP/IP data streams and ROS.
The REA client allows to publish the ANavS sensor fusion solution (mode `padsolution2ros`) and ANavS sensor data (mode `padsensordata2ros`) in ROS.

## Git clone including submodules and specific branch

`git clone --recursive -b <branch-name> https://vpn.anavs.de/anavs-development/vision/ros_ethernet_adapter.git`

## System requirements and dependencies

* OS: Linux/ Ubuntu (tested with Ubuntu 18.04, and Ubuntu 20.04)
* ROS base version installed (tested with ROS Melodic/ Ubuntu 18.04, and ROS Noetic/ Ubuntu 20.04)
  * Follow `http://wiki.ros.org/ROS/Installation` and install "ROS-Base: (Bare Bones)" version
* Additional Ubuntu packages:  
 ```
 ROS_VERSION=melodic
 sudo apt install ros-${ROS_VERSION}-tf2 ros-${ROS_VERSION}-tf2-ros ros-tf2-msgs libtf2-msgs-dev ros-${ROS_VERSION}-tf2-geometry-msgs libtf2-dev ros-${ROS_VERSION}-tf libxmlrpcpp-dev librosconsole-dev libactionlib-dev
 ```
 
## REA Client

### Prerequisites

1. Linux host with installed REA package.
2. ANavS MS-RTK module or Integrated Sensor Platform (ISP)
    * GNSS antennas connected (with sufficient GNSS reception)
    * Device powered on
3. Ethernet or WLAN connection between Linux host and ANavS system (Ethernet recommended)
    * Ethernet and WLAN settings can be configured in the ANavS Wizard
4. Start ANavS Wizard and configure the ANavS system
5. Start sensor fusion
    * A positioning solution should be provided (see 'Solution' tab in the ANavS GUI)

Copy the `ANAVS.conf` file into the REA subfolder `/configs`.
The file is usually stored on the MS-RTK module at: `/share/PAD/configs/ANAVS.conf`.
This is only required for mode `padsolution2ros`, for which specific information
is printed on the console and specific topics can be disabled.

To publish the ANavS sensor data only steps 1-3 are required.

### Set up your ROS environment

    ROS_VERSION=melodic
    source /opt/ros/${ROS_VERISON}/setup.bash
    roscore&

### Sensor fusion solution (mode: `padsolution2ros`)

Publishes ANavS sensor fusion solution in ROS. Assumes ANavS sensor fusion is running
(eg. on either the ANavS MS-RTK or ISP (remote host)) and TCP/IP data streams are available,
eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 6001). Specify the
remote host's IP for your setup (`--ip <host-ip>`), and ANavS sensor fusion solution port
if deviating from the default (`--port <port>`).

* Usage

 `anavs_ros_ethernet_client padsolution2ros [--ip <host-ip>] [--port <port>] [options]`

* Options:
 
   ```
    -h --help                             Show help.
    --ip <host-ip>                        IP address of remote host [default: 127.0.0.1].
    --port <host-port>                    TCP/IP data streams port. ANavS sensor fusion solution [default: 6001], sensor data [default: 4001].

    padsolution2ros:
    --topic_prefix <topic-prefix>         Prefix for all ROS topics published [default: /anavs/solution].
    --use_original_timestamps             Use original ANavS solution data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].
    --enable_odom                         Enables publishing odometry (topic: /odom; type: nav_msgs::Odometry) and angular rate (topic: /ang_rate; type: geometry_msgs/Vector3Stamped).
    --enable_pose                         Enables publishing pose (topic: /pose_enu; type: geometry_msgs::PoseWithCovarianceStamped).
    --odom_only                           Publish only the odometry message '/odom'.
    --pose_only                           Publish only the pose message '/pose_enu'.
    --publisher_queue_size <queue-size>   Sets ROS publisher queue sizes for all messages published. [default: 1]
    --pose_enu_frame <frame-id>           Specifies frame_id of published pose (topic: /pose_enu)' [default: map].
    --remove_pos_offset                   Enables removing initial position offset for publishing pose 'pose_enu', and odometry 'odom'.
    --publish_to_tf [--published_frame    Enables publishing transformation to ROS tf. Optional: Specify child_frame_id [default: base_link].
         <child-frame-id>]	
    --publish_path                        Enables publishing path (topic: /path; type: nav_msgs::Path) created from poses (topic: /pose_enu).
   ```

  * `--publisher_queue_size`
    * Default: `1`. For real-time settings. Old messages are dropped if they are still in the queue and cannot be published fast enough. Messages cannot queue up and delay publishing of messages.
    * Set `publisher_queue_size > 1` to avoid message drop. For post-processing settings, where all messages should be published and stored in favor of messages being published potentially delayed.
    
     For more details refer to [Choosing a good queue_size](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size).

* Use help to print all options:
 
 `./client/bin/Release/anavs_ros_ethernet_client --help`

* Advertised ROS topics:

    ```
     ROS topic                          Description                          ROS message type
     ------------------------------------------------------------------------------------------------------------------
     /anavs/solution/id                 ID                                   std_msgs/Uint8
     /anavs/solution/week               Week of current epoch                std_msgs/Uint16
     /anavs/solution/tow                Time of Week (sec)                   sensor_msgs/TimeReference
     /anavs/solution/pos                Position (NED* in meters)            geometry_msgs/PointStamped
     /anavs/solution/pos_llh            Position (lat,lon,height in          geometry_msgs/PointStamped
                                        deg,deg,meters)
     /anavs/solution/pos_xyz            Position (ECEF in meters)            geometry_msgs/PointStamped
     /anavs/solution/att_euler          Attitude/ Euler angles               geometry_msgs/PointStamped
                                        (heading, pitch, roll in rad)
     /anavs/solution/att_state          Attitude filter state                std_msgs/UInt8
     /anavs/solution/rtk_state          RTK filter state                     std_msgs/UInt8
     /anavs/solution/num_sats           Number of satellites                 std_msgs/UInt8
     /anavs/solution/imu_calibrated     IMU initialized/calibrated           std_msgs/Bool
     /anavs/solution/vel                Velocity (NED* in m/s)               geometry_msgs/Vector3Stamped
     /anavs/solution/acc                Acceleration (body frame in m/s^2)   geometry_msgs/Vector3Stamped
     /anavs/solution/pos_stddev         Std Dev of position (NED* in meters) geometry_msgs/PointStamped
     /anavs/solution/vel_stddev         Std Dev of velocity (NED* in m/s)    geometry_msgs/PointStamped
     /anavs/solution/att_euler_stddev   Std Dev of attitude/                 geometry_msgs/PointStamped
                                        Euler angles (in rad)
     /anavs/solution/accuracy           Estimated accuracy (m)               geometry_msgs/PointStamped
                                        (stored in: point.x)
     ------------------------------------------------------------------------------------------------------------------
     Optional:
     /anavs/solution/pose_enu           Pose (ENU in meters)                 geometry_msgs/PoseWithCovarianceStamped
     /anavs/solution/odom               Odometry (Pose: ENU in meters/       nav_msgs/Odometry
                                        Twist: body frame, velocity in m/s
                                        angular rate in rad/s)
     /anavs/solution/ang_rate           Angular rate (NED* in rad/s)         geometry_msgs/Vector3Stamped
     /path                              Path for visualization               nav_msgs/Path
     ------------------------------------------------------------------------------------------------------------------
     *): The default frame for 'pos', 'vel' and 'ang_rate' is NED, but it may vary if a specific customer code
         is used (see ANAVS Wizard: 'Customer-Code', and ANAVS.conf: 'customer_code').
         Also the body frame definition may vary if a specific customer code is used.
     ------------------------------------------------------------------------------------------------------------------
     The standard ROS topics provided correspond to the fields of the Standard Binary Solution Message
     documented in the User Reference Guide, section 6.1.:
     https://anavs.com/knowledgebase/user-reference-guide/
     In the ROS messages angular measurements are provided in 'rad' rather than 'deg'.
    ```

* Topics description:
        
    |ROS topic | Description |
    |----------|-------------|
    |id | Identifier of the ANavS Position and Attitude Determination (PAD) solution.|
    |week | Week number of the current epoch. |
    | tow | Time reference message containing the ROS time and the GNSS Time of Week (TOW) in seconds.   |
    | pos | The position in local coordinate frame (NED*) in meters. Baseline in local coordinate frame spanned by the position given by `pos_llh` in lat, lon and height, and by the position of the reference station. |
    | pos_llh  | The position in geographic coordinates (lat, lon, height) in degree, degree, meters.  |
    | pos_xyz  | The position in ECEF coordinates (X, Y, Z) in meters.  |
    | att_euler  | The attitude in Euler angles (heading, pitch, roll) in rad.  |
    | att_state  | The State-definition for the Attitude Kalman-filter: {0,1,2,3}  - 0: No solution, 1: Least-Squares solution, 2: Float solution (orange light in ANavS GUI), 3: Fixed solution (green light in ANavS GUI) |
    | rtk_state  | The State-definition for the RTK-Position Kalman-filter {0,1,2,3} - 0: No solution, 1: Least-Squares solution, 2: Float solution (orange light in ANavS GUI), 3: Fixed solution (green light in ANavS GUI) |
    | num_sats  | Number of satellites ( May be larger than the number of USED satellites, indicated in the ANavS GUI). |
    | imu_calibrated | Is set if the IMU is initialized/ calibrated. |
    | vel | The velocity in local coordinate frame (NED*) in m/s. |
    | acc | The acceleration in body frame in m/s^2. |
    | pos_stddev | The standard deviation of the position (NED*) in meters. |
    | vel_stddev | The standard deviation of the velocity (NED*) in m/s. |
    | att_stddev | The standard deviation of the attitude in Euler angles in rad. |
    | accuracy |Estimated accuracy of the position in local coordinate frame in meters. The scalar accuracy value is stored in the component `point.x`. Components `point.y` and `point.z` are set to `NaN`.|
    | Optional: | |
    |pose_enu   |  The pose in ENU frame in meters. The geographic north direction is aligned with ROS frame y-axis. Heading anlge 0° corresponds to north direction and results in ROS in an orientation where body-frame x-axis (red-axis in rviz) points into ROS frame y-direction. The 6x6 covariance matrix contains the variances of position X, Y, Z and rotation around X, Y, and Z on the diagonal. The covariances are set to zero currently. In case position variances contain NaN/Inf values, the covariance matrix left-upper 3x3 part is set to zeros. In case rotation variances contain NaN/Inf values, the covariance matrix right-lower 3x3 part is set zeros. |
    |odom | Pose (ENU) in meters, and twist (body frame), velocity in m/s and angular rate in rad/s. Pose covariance matrix, as described for `pose_enu`. Twist covariance matrix is set zero (not available). |    
    |ang_rate | Angular rate (NED*) in rad/s. |    
    | path | Path for visualization in RViz. |
    
     *): The default frame for 'pos', 'vel' and 'ang_rate' is NED, but it may vary if a specific customer code is used (see ANAVS Wizard: 'Customer-Code', and ANAVS.conf: 'customer_code'). Also the body frame definition may vary if a specific customer code is used.
     
     The standard ROS topics provided correspond to the fields of the Standard Binary Solution Message documented in the [User Reference Guide, section 6.1.](https://anavs.com/knowledgebase/user-reference-guide/)
     In the ROS messages angular measurements are provided in 'rad' rather than 'deg'.

* Remarks

  Please note that the positioning estimates (position, attitude, velocity and acceleration) potentially may jump, ie. they are not guaranteed to be continuous all the time. The nav_msgs/Odometry pose frame is thus set to "map" not "odom", to express that measurements are not guaranteed to be continuous.

* Visualization in RViz

  * Run the REA client with enabled options to publish pose and odom topics to publish to tf and to publish a path of the position trajectory. To start at zero position use the flag `--remove_pos_offset` to remove the initial position offset from the published positions in `pose_enu`:

  ```./client/bin/Release/anavs_ros_ethernet_client padsolution2ros [--ip <host-ip>] [--port <port>] --enable_pose --enable_odom --remove_pos_offset --publish_to_tf --publish_path [options]```

  * Run RViz using the provided config file:
  
 ```rviz -d configs/anavs.rviz```
 
    Following items are displayed:

  * PoseWithCovariance: /anavs/solution/pose_enu
  * Odometry: /anavs/solution/odom
  * Path: /path

### Sensor data (mode: `padsensordata2ros`)

Publishes ANavS sensor data in ROS. Assumes powered on ANavS MS-RTK or ISP (remote host)
(ANavS sensor fusion does not need to be running) and TCP/IP data streams are available,
eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 4001). Specify the
remote host's IP for your setup (`--ip <host-ip>`).

* Usage

 `./client/bin/Release/anavs_ros_ethernet_client padsensordata2ros [--ip <host-ip>] [--port <port>] [options]`
 
* Options:

  ```
  -h --help                             Show help.
  --ip <host-ip>                        IP address of remote host [default: 127.0.0.1].
  --port <host-port>                    TCP/IP data streams port. ANavS sensor fusion solution [default: 6001], sensor data [default: 4001].

  padsensordata2ros:
  --topic_prefix <topic-prefix>         Prefix for all ROS topics published [default: no prefix].
  --use_original_timestamps             Use original ANavS sensor data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].
  --publisher_queue_size <queue-size>   Sets ROS publisher queue sizes for all messages published. [default: 1]
  --imu_topic [<topic-name>]            Specifies IMU topic name [default: /imu0].
  --calibrate_imu                       Enables IMU calibration.
  --pub_imu_calibration [<topic-name>]  Enables publishing IMU calibration. Optional: Specify topic name [default: /imu_calibration].
  ```

  * `--calibrate_imu` Calibrates IMU at initialization. IMU must be static at initialization. Initial IMU measurements are used to calibrate the gyroscope bias and the accelerometer bias. A horizontal alignment of the IMU with z-axis pointing into gravity direction is assumed.
 
   * `--publisher_queue_size`
      * Default: `1`. For real-time settings. Old messages are dropped if they are still in the queue and cannot be published fast enough. Messages cannot queue up and delay publishing of messages.
      * Set `publisher_queue_size > 1` to avoid message drop. For post-processing settings, where all messages should be published and stored in favor of messages being published potentially delayed.
    
      For more details refer to [Choosing a good queue_size](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size).
 
 Use help to print all options:
 
 `./client/bin/Release/anavs_ros_ethernet_client --help`
 
* Advertised ROS topics:

  ```
  ROS topic                        Description                   ROS message type
  -------------------------------------------------------------------------------------------------------
  /imu0                            IMU messages                  sensor_msgs/Imu
  ```

* Topics description:

  * `imu0`: IMU messages, including angular velocity and linear acceleration measurements.

### Leap seconds from GNSS receiver (mode: `leapseconds2ros`)

Publishes leapseconds from GPS in ROS. Assumes powered on ANavS MS-RTK or ISP (remote host)
(ANavS sensor fusion does not need to be running) and TCP/IP data streams are available,
eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 4004). Specify the
remote host's IP for your setup (`--ip <host-ip>`).

* Usage

 `./client/bin/Release/anavs_ros_ethernet_client leapseconds2ros [--ip <host-ip>] [--port <port>] [options]`

* Options:
  ```
  -h --help                             Show help.
  --ip <host-ip>                        IP address of remote host [default: 127.0.0.1].
  --port <host-port>                    TCP/IP data streams port. ANavS sensor fusion solution [default: 6001], sensor data [default: 4001].
  
  leapseconds2ros:
  --use_original_timestamps             Use original ANavS sensor data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].
  --publisher_queue_size <queue-size>   Sets ROS publisher queue sizes for all messages published. [default: 1]
  ```

* Advertised ROS topics
  ```
  ROS topic                        Description                   ROS message type
  -------------------------------------------------------------------------------------------------------
  /leapseconds                     leap seconds from             std_msgs/Int8
                                   GPS receiver (second)
  ```

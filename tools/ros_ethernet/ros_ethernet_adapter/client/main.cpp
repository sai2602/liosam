/*
*   ANAVS ROS-Ethernet Client
*
*       Provides TCP/IP client functionality. Publishes ROS messages generated
*       from TCP/IP input streams. Configurations analogous to those of the
*       ANAVS ROS-Ethernet server are provided for testing.
*
*       Dependencies:
*           - ROS (for publishing ROS messages)
*/

/*
 * File:     client/main.cpp
 * Author:   R. Bensch, P. Faerber, J. Krause ANavS GmbH
 *
 * History:  2018/04/18 - Initial file creation
 *           2018/06/20 - Changed message type naming. Added message type
 *                        'PADFeeder_ROS'. Enabled receiving TCP/IP stream
 *                        from PAD feeder and publishing ROS messages.
 *           2018/07/13 - Added wheel odometry messages decoding and publishing
 *                        in deserialize_ubx2ROS. Dependent on the MSGID,
 *                        which is read from the data stream the decode_imu( ) or
 *                        the decode_wheel( ) function is called.
 *           2018/08/23 - Added additional publisher with the biases which are estimated
 *                        at the beginning of the client start. Subscribe in feeder
 *                        add it to the output of the MSF to compare the bias estimations.
 *           2018/09/20 - Hardcoded name for topic '/imu_calibration' is replaced by command
 *                        line argument 'rostopic3' in message type 'PADFeeder_ROS'.
 *           2018/09/24 - Odometry Msg includes IMU angular rates for misalignment of
 *                        IMU and Wheel odometry position
 *           2019/01/11 - For message type 'PADFeeder_ROS' added boolean {0,1} cmdline parameter
 *                        'decode_imu_callback'. Allows to disable calling the ros callback when
 *                        decoding imu data, which disables publishing imu and imu calibration
 *                        messages. Use in case publishing the time base only extracted from
 *                        imu data (time_towGNSS) is of interest, while avoiding to publish
 *                        irrelevant data.
 *	        2019/02/25 - Added publish /clock topic. Overwrites ROS::time::now with /clock time
 *           2019/02/26 - Added message type 'PADSolution_ROS'. Allows decoding of ANavS PAD
 *                        solution ubx messages. Publishes ROS messages containing PAD solution
 *                        information, such as solution position in different coordinate frames.
 *           2019/03/06 - Updated help text, added print_help() function. Added simple command
 *                        line parsing. Added version number via define 'VER_ROSETHCLI'.
 *                        Added client reconnect after connection interruption (client tries to
 *                        reconnect every 5sec, reconnect interval can be set via cmdline
 *                        parameter '--reconnection_delay=nsec'.; alternative reconnection
 *                        method via exponential backoff is commented).
 *           2019/03/12 - Merge Julius implementation/extension of 'PADSolution_ROS' mode for
 *                        publishing ANavS solution ROS messages from ubx solution stream.
 *                        Disabled pose_enu.orientation and covariance matrix rotation values,
 *                        orientation quaternion needs to be check again. Integrated
 *                        customer_code (2: AGT/STIHL), only mode=4 available. Updated and
 *                        tested only for mode=4 yet. TODO: Update and test for other modes too.
 *           2019/03/13 - v0.9.1: minor changes
 *           2019/04/08 - Merged two code versions: 1) Client integrated "odom" topic publishing
 *                        for Cartographer (by Paul), 2) Client integrated padsolution2ros mode
 *                        for publishing ANavS solution as ROS topics, client improvements
 *                        such as, automatic reconnet (by Julius, Robert)
 *           2019/04/10 - Fixed bug in deserialize_ubx2ROS that hindered publishing of tow messages.
 *                        Moved 'DEBUG_MODE' define to build target 'debug-exe' in project build
 *                        options.
 *           2019/06/03 - For mode padsensordata2ros, added flag to enable clock topic, added flag
 *                        '--use_imu_time' to switch between use of imu as time basis, or ROS time.
 *                        Renamed all functions and callback functions. Other minor adaptations.
 *           2019/08/14 - Updated ReadConfigFile().
 *           2019/10/28 - Read config parameters 'PAD-gear_ratio', 'PAD-wheel_radius' directly
 *                        from ANAVS.conf file. Added function ReadAnavsConfigFile().
 *           2019/10/29 - Corrected pose_enu orientation quaternion, currently only uses pad solution
 *                        heading angle. Added transformation broadcaster. Activated publishing pose_enu
 *                        path. Added corresponding command line parameters/ flags to mode
 *                        "padsolution2ros".
 *           2019/12/12 - Temporarily replaced function DecodeIMUUBXData() by
 *                        function DecodeIMUUBXDataAndPublishTime(). Enforces publishing of time messages
 *                        synchronously with IMU messages, to compare to previous implementation where
 *                        time messages are published after decoding when (k == nRead) holds. Observation
 *                        was that fewer time messages than IMU messages were generated.
 *           2019/12/13 - Temporarily deactivated imu_calibration message publishing, to check if this
 *                        influences frequency of time_towGNSS message publishing.
 *           2020/01/16 - Completed implementation of pose publishing in padsolution2ros mode, now
 *                        includes 3d attitude in pose orientation quaternion. Removed own quaternion
 *                        implementation code, now using tf2 conversions from ROS/tf2.
 *           2020/01/17 - Modified anavs solution output topics: remove "gnss_rcptn", "ready4refix_msg",
 *                        added: "imu_calibrated" (rescode bit 6), "att_state" (rescode bits 9 to 10).
 *                        Corrected 3d attitude quaternion, to correspond to Matlab quat2eul used in
 *                        pad software. Completed padsolution2ros mode. Added customer_code = 1 to
 *                        mask only padsolution2ros mode to provide ROS wrapper for ANavS PAD solution.
 *                        --> First release version for IIT.
 *           2020/09/30 - Mode padsolution2ros: In the convariance matrix of pose_enu the position, or
 *                        attitude variances are set to zero, if they contain nan of inf values.
 *           2020/11/09 - Mode padsensordata2ros: Add support for MSRTK CAN odometry messages, flag
 *                        '--pub_can_odom_twist'.
 *           2021/07/28 - Mode padsolution2ros: Rechecked conversion of NED position and Euler angles to
 *                        ENU ROS frame. Also checked values set in pose covariance matrix. Added
 *                        pos_ned, att_euler. Switched to using mutli protocol parser to decode
 *                        ubx solution, to profit from working checksum verification for larger
 *                        massages.
 *           2021/10/04 - Removed debug console output (wheel heading) in CAN wheel odometry decode function.
 *                        In client mode 'padsolution2ros' corrected accuracy message output from type uint8 to
 *                        double type using geometry message pointstamped type. Updated print help function.
 *
 * Todo:    - Cmdline interface: Robustify command line interface, e.g.: Report usage of not existing parameters, disallow
 *            to omit value for certain parameters, just allow for combined flag/value parameters.
 *
 */

#define VER_ROSETHCLI "0.9.16"    /* ros ethernet adapter, tcp-client mode */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <fstream>      // std::ofstream
#include <string>
#include <array>

#include <algorithm>

#include <unistd.h>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
//#include "rosgraph_msgs/Clock.h"
#include "nav_msgs/Odometry.h"

#include "tf2_ros/transform_broadcaster.h"
//#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/buffer_core.h>
#include "nav_msgs/Path.h"

// TCP sockets
#include "../sockets/definition.h"
#include "../sockets/ClientSocket.h"
#include "../sockets/SocketException.h"

//ROS and Maplab-PAD type definitions
#include "../ros_types.h"
#include "../maplab_PAD_types.h"
#include "../ubx_types.h"
//#include "../ublox_partial.c"

#include "../common.h"

#include "../decoder-cpp/src/MultiProtocolParser.h"
#include "../decoder-cpp/src/LambdaAdapter.h"
#include "../decoder-cpp/src/UbxSubparser.h"
#include "../decoder-cpp/src/UbxPackets.h"
#include "../decoder-cpp/src/ubx_parser.h"

#ifdef DEBUG_MODE
#include "../debug_helper.h"
#endif // DEBUG_MODE

#define MODE_UNDEFINED -1
#define MODE_DEBUG_PADFEEDER_VIO 0
#define MODE_DEBUG_PADGUI_IMAGE 1
#define MODE_PADSENSORDATA2ROS 2
#define MODE_PADSOLUTION2ROS 3
#define MODE_LEAPSECONDS2ROS 4

#define IMU_STANDARD 1      // low-cost IMU
#define IMU_OPTIONAL 2      // industrial-grade IMU

using namespace std;

static bool use_original_timestamps = false;
//static bool clock_enabled = false;
// current imu timestamp
static bool overwrite_odo_tow_by_imu_tow = false;
static uint64_union imu_towGNSS = {0};
static bool pub_time_imu_tow = false;

// IMU scale factors, gyro and accelerometer calibration
static int imu_ident = 0;  // 0: no imu data, 1: IMU_STANDARD, 2: IMU_OPTIONAL (and IMU_STANDARD),
static bool imu_ident_detected = false;
static bool optimu_disabled = false;
static double detect_optimu_timeout_sec = 1.;

struct ImuScalingFactors {
    double gyro_scale;  // gyro scaling to rad/s
    double acc_scale;   // accelerometer scaling to m/s^2
};
// Low-cost IMU
static const ImuScalingFactors scale_factors_imu = { .gyro_scale = 1./131.0 * M_PI/180.,        //0.007633588 * M_PI/180
                                                     .acc_scale = 1./16384.0 * 9.81 };          //0.0005987549
// Optional IMU
static const ImuScalingFactors scale_factors_optimu = { .gyro_scale = 0.005 * M_PI/180.,
                                                        .acc_scale = 25./100000.0 * 9.81 };
static bool imu_calibration = false;
static bool pub_imu_calibration = false;
static bool decode_imu_callback = true;

static bool decode_odometer_data = false;
static bool decode_can_odom_data = false;
static bool pub_odom_twist = false;
static bool pub_odom_odometry = false;
static bool pub_can_odom_twist = false;

static int calibrMsgCount = 0;
static const int calibrMsgTotal = 200;
static const double gravity_magnitude_mps2 = 9.81;
static const double calibrPriorBiasAcc_x = 0, calibrPriorBiasAcc_y = 0, calibrPriorBiasAcc_z = -gravity_magnitude_mps2;
//static const double calibrPriorBiasAcc_x = 0, calibrPriorBiasAcc_y = 0, calibrPriorBiasAcc_z = +gravity_magnitude_mps2;     // applies for optional IMU
static double sumGyro_x = 0, sumGyro_y = 0, sumGyro_z = 0;
static double sumAcc_x = 0, sumAcc_y = 0, sumAcc_z = 0;
static double biasGyro_x = 0, biasGyro_y = 0, biasGyro_z = 0;
static double biasAcc_x = 0, biasAcc_y = 0, biasAcc_z = 0;
static double angular_velocity_x = 0, angular_velocity_y = 0, angular_velocity_z = 0;

sMaplabPADFeeder *msg_PADFeeder = NULL;
sMaplabPADGUI *msg_PADGUI = NULL;
sPADFeederMaplab *msg_Maplab = NULL;
sPADSolution *msg_Solution = NULL;

// padsolution
static int pad_customer_code = 0;           // PAD customer code (default: 0, contained in ANAVS.conf file)

// frames
const std::string att_euler_frame("world");
const std::string pos_llh_frame("world");
const std::string pos_xyz_frame("world");
const std::string pos_ned_frame("map");
const std::string accuracy_frame("map");
std::string pose_enu_frame("map");
const std::string vel_ned_frame("map");
const std::string ang_rate_frame("map");
const std::string sensorfusion_body_frame("base_link");
const std::string acc_frame(sensorfusion_body_frame);       // sensorfusion body frame, but acceleration measurements based on IMU position
std::string published_frame("base_link");
// flags
bool publish_to_tf = false;
bool publish_path = false;
bool remove_pos_offset = false;
bool init_pos_set = false;
bool enable_odom = false;
bool enable_pose = false;
bool odom_only = false;
bool pose_only = false;
bool decode_filters = false;
double init_pos_x = 0.;
double init_pos_y = 0.;
double init_pos_z = 0.;

// padsensordata
// frames
const std::string imu_frame("imu_link");

// ROS publisher
const int kLatestOnlyPublisherQueueSize = 1;    // for real-time requirements, no old messages should queue up and be published, instead older queue messages be dropped
const int kPublisherQueueSizePath = 10;
int kPublisherQueueSize = kLatestOnlyPublisherQueueSize;
ros::Publisher pub_id;
ros::Publisher pub_week;
ros::Publisher pub_imu;
//ros::Publisher pub_clock;
ros::Publisher pub_tow;
ros::Publisher pub_wheel;
ros::Publisher pub_imu_calibr;
ros::Publisher pub_pose_enu;
ros::Publisher pub_pose_twist_enu;
ros::Publisher pub_position_ned;    // NED frame
ros::Publisher pub_pos_ned_stddev;
ros::Publisher pub_position_llh;    // Geodetic frame
ros::Publisher pub_position_xyz;    // ECEF frame
ros::Publisher pub_sol_tow;
ros::Publisher pub_att_euler;
ros::Publisher pub_att_euler_stddev;
ros::Publisher pub_att_state;
ros::Publisher pub_vel_ned;
ros::Publisher pub_vel_ned_stddev;
ros::Publisher pub_acc;
//ros::Publisher pub_acc_stddev;
ros::Publisher pub_rtk_state;
ros::Publisher pub_num_sat;
ros::Publisher pub_imu_calibrated;
ros::Publisher pub_accuracy;
//ros::Publisher pub_ready4refix;
//ros::Publisher pub_gnss_rcptn;
ros::Publisher pub_odom;
ros::Publisher pub_can_odom;
ros::Publisher pub_ang_rate;

ros::Publisher pub_leapseconds;

ros::Publisher pub_path;
nav_msgs::Path path;

void VIOUBXData_callback(const sMaplabPADFeeder& msg)
{

#ifdef DEBUG_MODE
    cout << "\n*** RECEIVED MESSAGE ***\n";
    cout << "msg.odometryB.header.seq: " << +msg.odometryB.header.seq << endl;
    cout << "msg.odometryB.header.stamp.sec: " << +msg.odometryB.header.stamp.sec << endl;
    cout << "msg.odometryB.header.stamp.nsec: " << +msg.odometryB.header.stamp.nsec << endl;
    cout << "msg.odometryB.twist.twist.linear.x: " << +msg.odometryB.twist.twist.linear.x << endl;
    cout << "msg.odometryB.twist.twist.linear.y: " << +msg.odometryB.twist.twist.linear.y << endl;
    cout << "msg.odometryB.twist.twist.linear.z: " << +msg.odometryB.twist.twist.linear.z << endl;
    cout << "msg.odometryB.twist.twist.angular.x: " << +msg.odometryB.twist.twist.angular.x << endl;
    cout << "msg.odometryB.twist.twist.angular.y: " << +msg.odometryB.twist.twist.angular.y << endl;
    cout << "msg.odometryB.twist.twist.angular.z: " << +msg.odometryB.twist.twist.angular.z << endl;
    cout << "msg.odometryB.twist.covariance.linear: ";
    for (int i = 0; i < 9; ++i)
        cout << +msg.odometryB.twist.covariance.linear[i] << " ";
    cout << endl;
    cout << "msg.odometryB.twist.covariance.angular: ";
    for (int i = 0; i < 9; ++i)
        cout << +msg.odometryB.twist.covariance.angular[i] << " ";
    cout << endl;
    cout << "msg.biasAcc.x: " << +msg.biasAcc.x << endl;
    cout << "msg.biasAcc.y: " << +msg.biasAcc.y << endl;
    cout << "msg.biasAcc.z: " << +msg.biasAcc.z << endl;
    cout << "msg.biasGyro.x: " << +msg.biasGyro.x << endl;
    cout << "msg.biasGyro.y: " << +msg.biasGyro.y << endl;
    cout << "msg.biasGyro.z: " << +msg.biasGyro.z << endl;
    cout << "msg.pose.position.x: " << +msg.pose.position.x << endl;
    cout << "msg.pose.position.y: " << +msg.pose.position.y << endl;
    cout << "msg.pose.position.z: " << +msg.pose.position.z << endl;
    cout << "msg.pose.orientation.x: " << +msg.pose.orientation.x << endl;
    cout << "msg.pose.orientation.y: " << +msg.pose.orientation.y << endl;
    cout << "msg.pose.orientation.z: " << +msg.pose.orientation.z << endl;
    cout << "msg.pose.orientation.w: " << +msg.pose.orientation.w << endl;
    cout << "msg.quality: " << +msg.quality << endl;
#endif // DEBUG_MODE

    // DEBUG measurements
    //EvalPADFeederMsgs( msg);

}

void ImageUBXData_callback(const sMaplabPADGUI& msg)
{

#ifdef DEBUG_MODE
    cout << "\n*** RECEIVED MESSAGE ***\n";
    cout << "msg.image.header.seq: " << +msg.image.header.seq << endl;
    cout << "msg.image.header.stamp.sec: " << +msg.image.header.stamp.sec << endl;
    cout << "msg.image.header.stamp.nsec: " << +msg.image.header.stamp.nsec << endl;
    cout << "msg.image.height: " << +msg.image.height << endl;
    cout << "msg.image.width: " << +msg.image.width << endl;
    cout << "msg.image.is_bigendian: " << +msg.image.is_bigendian << endl;
    cout << "msg.image.step: " << +msg.image.step << endl;

    cout << "msg.image.data: " << " : " << (msg.image.height * msg.image.step) << endl;
    for (int i = 0; i < 20; ++i) {
        cout << +msg.image.data[i] << " ";
    }
    cout << endl;

    // debug - write MONO8 image data as PGM file
    /*
    ostringstream seq_s;
    seq_s << std::setw( 4 ) << std::setfill( '0' ) << (int)msg.image.header.seq;
    ostringstream s;
    s << "/media/robert/Shared/Data/test/img" << seq_s.str() << ".pgm";
    string filename(s.str());
    WriteImagePGM( (unsigned char *)msg.image.data, msg.image.width, msg.image.height,
                    filename);
    */

    // debug - write RGB8 image data as PPM file
    /*
    ostringstream seq_s;
    seq_s << std::setw( 4 ) << std::setfill( '0' ) << (int)msg.image.header.seq;
    ostringstream s;
    s << "/media/robert/Shared/Data/test/img" << seq_s.str() << ".ppm";
    string filename(s.str());
    WriteImagePPM( (unsigned char *)msg.image.data, msg.image.width, msg.image.height,
                    filename);
    */
#endif // DEBUG_MODE

}


void SolutionUBXData_callback(const UbxPadSol& msg)
{
#ifdef DEBUG_MODE
    cout << "\n*** RECEIVED MESSAGE ***\n";
    cout.precision(17);
    cout << "msg.id: " << +msg.id << endl;
    cout << "msg.resCode: " << +msg.resCode << endl;
    cout << "msg.week: " << +msg.week << endl;
    cout << "msg.tow: " << msg.tow << endl;
    cout << "msg.weekInit: " << +msg.weekInit << endl;
    cout << "msg.towInit: " << msg.towInit << endl;
    cout << "msg.lat: " << msg.lat << endl;
    cout << "msg.lon: " << msg.lon << endl;
    cout << "msg.height: " << msg.height << endl;
    cout << "msg.ECEF_X: " << msg.ECEF_X << endl;
    cout << "msg.ECEF_Y: " << msg.ECEF_Y << endl;
    cout << "msg.ECEF_Z: " << msg.ECEF_Z << endl;
    cout << "msg.att[0]: " << msg.att[0] << endl;
    cout << "msg.att[1]: " << msg.att[1] << endl;
    cout << "msg.att[2]: " << msg.att[2] << endl;
    cout << "msg.attStdDev[0]: " << msg.attStdDev[0] << endl;
    cout << "msg.attStdDev[1]: " << msg.attStdDev[1] << endl;
    cout << "msg.attStdDev[2]: " << msg.attStdDev[2] << endl;
    cout << "msg.accuracy: " << msg.accuracy << endl;
    cout << "msg.systemTime: " << msg.systemTime << endl;
    cout << "msg.prePAD: " << msg.prePAD << endl;
    cout << "msg.postPAD: " << msg.postPAD << endl;
    cout << "msg.latency: " << msg.latency << endl;
    cout << "msg.accNed[0]: " << msg.accNed[0] << endl;
    cout << "msg.accNed[1]: " << msg.accNed[1] << endl;
    cout << "msg.accNed[2]: " << msg.accNed[2] << endl;
    cout << "msg.numSats: " << unsigned(msg.numSats) << endl;
    cout << "msg.numRcv: " << unsigned(msg.numRcv) << endl;
    cout << "msg.numSatsMeas: " << unsigned(msg.numSatsMeas) << endl;
    cout << "msg.numBl: " << unsigned(msg.numBl) << endl;
    cout << "msg.numFilter: " << unsigned(msg.numFilter) << endl;
    cout << "msg.angRate[0]: " << msg.angRate[0] << endl;
    cout << "msg.angRate[1]: " << msg.angRate[1] << endl;
    cout << "msg.angRate[2]: " << msg.angRate[2] << endl;
    cout << "msg.angRateStdDev[0]: " << msg.angRateStdDev[0] << endl;
    cout << "msg.angRateStdDev[1]: " << msg.angRateStdDev[1] << endl;
    cout << "msg.angRateStdDev[2]: " << msg.angRateStdDev[2] << endl;

#endif // DEBUG_MODE

    // set messages timestamp at beginning of callback function
    const ros::Time msg_stamp_ros_time_now = ros::Time::now();
    const ros::Time msg_stamp_tow = ros::Time(msg.tow);

    const ros::Time msg_stamp = (use_original_timestamps == true) ? msg_stamp_tow : msg_stamp_ros_time_now;

//    ros::Time msg_stamp;
//    if (use_original_timestamps) {
//        // tow
//        msg_stamp = msg_stamp_tow;
//    } else {
//        // ROS time
//        msg_stamp = msg_stamp_ros_time_now;
//    }

    // transformation given by PAD software: from frame NED to body
    // position (NED frame, in meters)
    const double& pos_N = msg.b[0];  // baseline N
    const double& pos_E = msg.b[1];  // baseline E
    const double& pos_D = msg.b[2];  // baseline D

    // 3d attitude heading/ yaw, pitch, roll
    //  in PAD software computed from orientation q_nb (from body to NED)
    //  using Matlab function quat2eul(), standard rotation order 'ZYX', ie. heading, pitch, roll
    // convert attitude (euler angles in deg in NED frame) to rad
    const double att_head = msg.att[0] * toRad;         // attitude heading
    const double att_pitch = msg.att[1] * toRad;        // attitude pitch
    const double att_roll = msg.att[2] * toRad;         // attitude roll

    // velocity in NED
    const double& vel_N = msg.vel[0];
    const double& vel_E = msg.vel[1];
    const double& vel_D = msg.vel[2];

    // covariance matrix of position and orientation
    //  6x6 matrix, only diagonal elements set
    //  position, or attitude variances are set to zero, if
    //  the contain nan of inf values

    // position standard deviations
    const double& pos_N_std = msg.bStdDev[0];    // baseline N stdev
    const double& pos_E_std = msg.bStdDev[1];    // baseline E stdev
    const double& pos_D_std = msg.bStdDev[2];    // baseline D stdev

    // attitude standard deviations in rad
    const double att_head_std = msg.attStdDev[0] * toRad;       // attitude Heading stdev
    const double att_pitch_std = msg.attStdDev[1] * toRad;      // attitude Pitch stdev
    const double att_roll_std = msg.attStdDev[2] * toRad;       // attitude Roll stdev

    // velocity standard deviations
    const double& vel_N_std = msg.velStdDev[0];
    const double& vel_E_std = msg.velStdDev[1];
    const double& vel_D_std = msg.velStdDev[2];

    // check position and attitude for NaN or Inf values
    const bool pos_not_nan_or_inf = std::isfinite(pos_N) &&
                                    std::isfinite(pos_E) &&
                                    std::isfinite(pos_D);
    const bool att_not_nan_or_inf = std::isfinite(att_head) &&
                                    std::isfinite(att_pitch) &&
                                    std::isfinite(att_roll);

    // check position and attitude standard deviatons for NaN or Inf values
    const bool pos_std_not_nan_or_inf = std::isfinite(pos_N_std) &&
                                        std::isfinite(pos_E_std) &&
                                        std::isfinite(pos_D_std);

    const bool att_std_not_nan_or_inf = std::isfinite(att_head_std) &&
                                        std::isfinite(att_pitch_std) &&
                                        std::isfinite(att_roll_std);

    // angular rate in NED
    // only contains valid information if enable_odom == true
    const double ang_rate_N = msg.angRate[0] * toRad;
    const double ang_rate_E = msg.angRate[1] * toRad;
    const double ang_rate_D = msg.angRate[2] * toRad;

    // check angular rate for nan values
    const bool ang_rate_not_nan_or_inf = std::isfinite(ang_rate_N) &&
                                        std::isfinite(ang_rate_E) &&
                                        std::isfinite(ang_rate_D);

    // ROS messages from standard pad solution
    if (!odom_only && !pose_only) {

        const uint8_t& id = msg.id;
        const uint16_t& week = msg.week;

        const double& pos_lat = msg.lat;
        const double& pos_lon = msg.lon;
        const double& pos_height = msg.height;

        const double& pos_x_ECEF = msg.ECEF_X;
        const double& pos_y_ECEF = msg.ECEF_Y;
        const double& pos_z_ECEF = msg.ECEF_Z;

        // acceleration in body frame
        const double& acc_x = msg.acc[0];
        const double& acc_y = msg.acc[1];
        const double& acc_z = msg.acc[2];

        // acceleration standard deviations
        //  disabled, is set to NaN in PAD software
        //const double& acc_x_std = msg.accStdDev[0];
        //const double& acc_y_std = msg.accStdDev[1];
        //const double& acc_z_std = msg.accStdDev[2];

        // attitude- and rtk-filter state
        //  00 --> No solution
        //  01 --> Least-Squares solution
        //  10 --> Float solution
        //  11 --> Fixed solution
        uint8_t att_state = 0;
        // extract bit 9 to 10 from resCode (state-definition for the Attitude Kalman-Filter)
        if( msg.resCode & ( 0x01 << 9 ) ) {
            att_state += 1;
        }
        if( msg.resCode & ( 0x01 << 10 ) ) {
            att_state += 2;
        }

        uint8_t rtk_state = 0;
        // extract bit 11 to 12 from resCode (state-definition for the RTK-Position Kalman-Filter)
        if( msg.resCode & ( 0x01 << 11 ) ) {
            rtk_state += 1;
        }
        if( msg.resCode & ( 0x01 << 12 ) ) {
            rtk_state += 2;
        }

        bool imu_calibrated = false;
        // extract bit 6 from resCode
        if( msg.resCode & ( 0x01 << 6 ) ) {
            imu_calibrated = true;
        }

        // ID
        std_msgs::UInt8 id_msg;
        id_msg.data = id;

        // week
        std_msgs::UInt16 week_msg;
        week_msg.data = week;

        // accuracy (of baseline)
        geometry_msgs::PointStamped accuracy_msg;
        accuracy_msg.header.stamp = msg_stamp;
        accuracy_msg.header.frame_id = accuracy_frame;
        accuracy_msg.point.x = msg.accuracy;
        setNaN(accuracy_msg.point.y);
        setNaN(accuracy_msg.point.z);

        // time reference message
        sensor_msgs::TimeReference tow_msg;
        tow_msg.header.stamp = msg_stamp_ros_time_now;  // ROS time
        //tow_msg.header.frame_id = '';                 // frame_id is not used
        tow_msg.time_ref = msg_stamp_tow;               // tow
        tow_msg.source = "anavs tow";

        // attitude filter state
        std_msgs::UInt8 att_state_msg;
        att_state_msg.data = att_state;

        // rtk filter state
        std_msgs::UInt8 rtk_state_msg;
        rtk_state_msg.data = rtk_state;

        // number of satellites
        std_msgs::UInt8 num_sat_msg;        // number of visible satellites (not used satellites, indicated in GUI skyplot)
        num_sat_msg.data = msg.numSats;

        // imu calibrated
        std_msgs::Bool imu_calibrated_msg;
        imu_calibrated_msg.data = imu_calibrated;

        // attitude Euler angles (in rad)
        geometry_msgs::PointStamped att_euler_msg;
        att_euler_msg.header.stamp = msg_stamp;
        att_euler_msg.header.frame_id = att_euler_frame;
        att_euler_msg.point.x = att_head;
        att_euler_msg.point.y = att_pitch;
        att_euler_msg.point.z = att_roll;

        // position NED
        geometry_msgs::PointStamped pos_ned_msg;
        pos_ned_msg.header.stamp = msg_stamp;
        pos_ned_msg.header.frame_id = pos_ned_frame;
        pos_ned_msg.point.x = pos_N;
        pos_ned_msg.point.y = pos_E;
        pos_ned_msg.point.z = pos_D;

        // position LLH (Lat, Lon, Height)
        geometry_msgs::PointStamped pos_llh_msg;
        pos_llh_msg.header.stamp = msg_stamp;
        pos_llh_msg.header.frame_id = pos_llh_frame;
        pos_llh_msg.point.x = pos_lat;
        pos_llh_msg.point.y = pos_lon;
        pos_llh_msg.point.z = pos_height;

        // position ECEF
        geometry_msgs::PointStamped pos_xyz_msg;
        pos_xyz_msg.header.stamp = msg_stamp;
        pos_xyz_msg.header.frame_id = pos_xyz_frame;
        pos_xyz_msg.point.x = pos_x_ECEF;
        pos_xyz_msg.point.y = pos_y_ECEF;
        pos_xyz_msg.point.z = pos_z_ECEF;

        // velocity NED
        geometry_msgs::Vector3Stamped vel_ned_msg;
        vel_ned_msg.header.stamp = msg_stamp;
        vel_ned_msg.header.frame_id = vel_ned_frame;
        vel_ned_msg.vector.x = vel_N;
        vel_ned_msg.vector.y = vel_E;
        vel_ned_msg.vector.z = vel_D;

        // acceleration body frame
        geometry_msgs::Vector3Stamped acc_msg;
        acc_msg.header.stamp = msg_stamp;
        acc_msg.header.frame_id = acc_frame;
        acc_msg.vector.x = acc_x;
        acc_msg.vector.y = acc_y;
        acc_msg.vector.z = acc_z;

        // position NED stddev
        geometry_msgs::PointStamped pos_ned_stddev_msg;
        pos_ned_stddev_msg.header.stamp = msg_stamp;
        pos_ned_stddev_msg.header.frame_id = pos_ned_frame;
        pos_ned_stddev_msg.point.x = pos_N_std;
        pos_ned_stddev_msg.point.y = pos_E_std;
        pos_ned_stddev_msg.point.z = pos_D_std;

        // velocity NED stddev
        geometry_msgs::PointStamped vel_ned_stddev_msg;
        vel_ned_stddev_msg.header.stamp = msg_stamp;
        vel_ned_stddev_msg.header.frame_id = vel_ned_frame;
        vel_ned_stddev_msg.point.x = vel_N_std;
        vel_ned_stddev_msg.point.y = vel_E_std;
        vel_ned_stddev_msg.point.z = vel_D_std;

        // acceleration stddev
        //geometry_msgs::PointStamped acc_stddev_msg;
        //acc_stddev_msg.header.stamp = msg_stamp;
        //acc_stddev_msg.header.frame_id = acc_frame;
        //acc_stddev_msg.point.x = acc_x_std;
        //acc_stddev_msg.point.y = acc_y_std;
        //acc_stddev_msg.point.z = acc_z_std;

        // attitude Euler angles stddev
        geometry_msgs::PointStamped att_euler_stddev_msg;
        att_euler_stddev_msg.header.stamp = msg_stamp;
        att_euler_stddev_msg.header.frame_id = att_euler_frame;
        att_euler_stddev_msg.point.x = att_head_std;
        att_euler_stddev_msg.point.y = att_pitch_std;
        att_euler_stddev_msg.point.z = att_roll_std;

        // publish all messages
        pub_id.publish(id_msg);
        pub_week.publish(week_msg);
        pub_position_ned.publish(pos_ned_msg);
        pub_position_llh.publish(pos_llh_msg);
        pub_position_xyz.publish(pos_xyz_msg);
        pub_sol_tow.publish(tow_msg);
        pub_att_euler.publish(att_euler_msg);
        pub_att_state.publish(att_state_msg);
        pub_rtk_state.publish(rtk_state_msg);
        pub_num_sat.publish(num_sat_msg);
        pub_imu_calibrated.publish(imu_calibrated_msg);
        pub_vel_ned.publish(vel_ned_msg);
        pub_acc.publish(acc_msg);
        pub_pos_ned_stddev.publish(pos_ned_stddev_msg);
        pub_vel_ned_stddev.publish(vel_ned_stddev_msg);
        //pub_acc_stddev.publish(acc_stddev_msg);
        pub_att_euler_stddev.publish(att_euler_stddev_msg);
        pub_accuracy.publish(accuracy_msg);

        // publish only if odom is enabled and filter loop decoded
        if (enable_odom) {
            // angular rate (from PAD solution filter loop)
            geometry_msgs::Vector3Stamped ang_rate_msg;
            ang_rate_msg.header.stamp = msg_stamp;
            ang_rate_msg.header.frame_id = ang_rate_frame;
            ang_rate_msg.vector.x = ang_rate_N;
            ang_rate_msg.vector.y = ang_rate_E;
            ang_rate_msg.vector.z = ang_rate_D;

            pub_ang_rate.publish(ang_rate_msg);
        }

    }

     // Derived ROS pose message
    geometry_msgs::PoseWithCovarianceStamped pose_enu_msg;
    if (enable_pose || enable_odom || publish_path || publish_to_tf) {
        pose_enu_msg.header.stamp = msg_stamp;
        pose_enu_msg.header.frame_id = pose_enu_frame;

        // conversion NED to ENU (ROS frame)
        convertNED2Pose(pos_N, pos_E, pos_D, att_head, att_pitch, att_roll,
                            pose_enu_msg.pose.pose);

        // option to remove initial position offset from output position
        if (!init_pos_set && pos_not_nan_or_inf) {
            init_pos_x = pose_enu_msg.pose.pose.position.x;
            init_pos_y = pose_enu_msg.pose.pose.position.y;
            init_pos_z = pose_enu_msg.pose.pose.position.z;
            init_pos_set = true;
            if (remove_pos_offset) {
                cout << "Initial position set (ENU in m): " << init_pos_x << ", " << init_pos_y << ", " << init_pos_z << endl;
                cout << "Removing offset from position (pose_enu): " << "enabled" << endl;
            }
        }
        // removes initial position offset from output positions
        if (remove_pos_offset) {
            pose_enu_msg.pose.pose.position.x -= init_pos_x;
            pose_enu_msg.pose.pose.position.y -= init_pos_y;
            pose_enu_msg.pose.pose.position.z -= init_pos_z;
        }

        // variances of position XYZ/ ENU
        //  stays zero if values are nan or inf
        if (pos_std_not_nan_or_inf) {
            pose_enu_msg.pose.covariance[0]  = pow(pos_E_std,2);
            pose_enu_msg.pose.covariance[7]  = pow(pos_N_std,2);
            pose_enu_msg.pose.covariance[14] = pow(pos_D_std,2);
        }

        // variances of rotation around XYZ/ roll, pitch, heading)
        if (att_std_not_nan_or_inf) {
            pose_enu_msg.pose.covariance[21] = pow(att_roll_std,2);  // Rotation around X - roll
            pose_enu_msg.pose.covariance[28] = pow(att_pitch_std,2); // Rotation around Y - pitch
            pose_enu_msg.pose.covariance[35] = pow(att_head_std,2);  // Rotation around Z - heading
        }
    }

    if (enable_pose && pos_not_nan_or_inf) {
        pub_pose_enu.publish(pose_enu_msg);
    }

    // publish to tf
    // publish position trajectory path
    if (publish_to_tf && pos_not_nan_or_inf && att_not_nan_or_inf) {
        static tf2_ros::TransformBroadcaster tr_br;
        // publish transform: laser --> map
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = msg_stamp;
        transformStamped.header.frame_id = pose_enu_frame;
        transformStamped.child_frame_id = published_frame;
        transformStamped.transform.translation.x = pose_enu_msg.pose.pose.position.x;
        transformStamped.transform.translation.y = pose_enu_msg.pose.pose.position.y;
        transformStamped.transform.translation.z = pose_enu_msg.pose.pose.position.z;
        transformStamped.transform.rotation.x = pose_enu_msg.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = pose_enu_msg.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = pose_enu_msg.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = pose_enu_msg.pose.pose.orientation.w;
        tr_br.sendTransform(transformStamped);
    }

    // publish trajectory path for vizalization
    if (publish_path && pos_not_nan_or_inf) {
        geometry_msgs::PoseStamped current_pose_msg;
        current_pose_msg.header = pose_enu_msg.header;
        current_pose_msg.pose = pose_enu_msg.pose.pose;
        path.header = pose_enu_msg.header;
        path.poses.push_back(current_pose_msg);
        pub_path.publish(path);
    }

    // Derived ROS nav odometry message
    if (enable_odom && pos_not_nan_or_inf && att_not_nan_or_inf) {

        // transformation velocity (from ENU to base_link frame)
        geometry_msgs::TransformStamped ts_lookup;
        geometry_msgs::TransformStamped tnb;
        geometry_msgs::TransformStamped t_ned_angRate_lookup;
        tnb.header.stamp = msg_stamp;
        tnb.header.frame_id = sensorfusion_body_frame;
        tnb.child_frame_id = pose_enu_frame;
        tnb.transform.translation.x = 0;
        tnb.transform.translation.y = 0;
        tnb.transform.translation.z = 0;
        tnb.transform.rotation.x = pose_enu_msg.pose.pose.orientation.x;
        tnb.transform.rotation.y = pose_enu_msg.pose.pose.orientation.y;
        tnb.transform.rotation.z = pose_enu_msg.pose.pose.orientation.z;
        tnb.transform.rotation.w = pose_enu_msg.pose.pose.orientation.w;
        tf2::Quaternion quat_tf;
        tf2::fromMsg(tnb.transform.rotation, quat_tf);
        tf2::Quaternion quat_tf_inv = quat_tf.inverse();
        tnb.transform.rotation = tf2::toMsg(quat_tf_inv);

        tf2::BufferCore buffer_core;
        buffer_core.setTransform(tnb, "default_authority");

        geometry_msgs::TransformStamped tnv;
        tnv.header.stamp = msg_stamp;
        tnv.header.frame_id = pose_enu_frame;
        tnv.child_frame_id = "target";
        tnv.transform.translation.x = vel_E;
        tnv.transform.translation.y = vel_N;
        tnv.transform.translation.z = -vel_D;
        tf2::Quaternion q0(0,0,0);
        tnv.transform.rotation = tf2::toMsg(q0);

        buffer_core.setTransform(tnv, "default_authority");

        ts_lookup = buffer_core.lookupTransform(sensorfusion_body_frame, "target", msg_stamp);

        //ROS_INFO("Velocity in NED: %f, %f, %f", vel_N, vel_E, vel_D);
        //ROS_INFO("%f, %f, %f", ts_lookup.transform.translation.x, ts_lookup.transform.translation.y, ts_lookup.transform.translation.z);
        //ROS_INFO("%f, %f, %f, %f", ts_lookup.transform.rotation.x, ts_lookup.transform.rotation.y, ts_lookup.transform.rotation.z, ts_lookup.transform.rotation.w);

        // ROS odometry message
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.stamp = msg_stamp;
        odometry_msg.header.frame_id = pose_enu_frame;
        odometry_msg.child_frame_id = sensorfusion_body_frame;
        odometry_msg.pose = pose_enu_msg.pose;
        odometry_msg.twist.twist.linear.x = ts_lookup.transform.translation.x;
        odometry_msg.twist.twist.linear.y = ts_lookup.transform.translation.y;
        odometry_msg.twist.twist.linear.z = ts_lookup.transform.translation.z;
        // transform angular rate (from NED to body frame)
        if (ang_rate_not_nan_or_inf){
            geometry_msgs::TransformStamped t_ned_angRate;
            t_ned_angRate.header.stamp = msg_stamp;
            t_ned_angRate.header.frame_id = pose_enu_frame;
            t_ned_angRate.child_frame_id = "angrate";
            t_ned_angRate.transform.translation.x = ang_rate_E;
            t_ned_angRate.transform.translation.y = ang_rate_N;
            t_ned_angRate.transform.translation.z = -ang_rate_D;
            t_ned_angRate.transform.rotation = tf2::toMsg(q0);

            buffer_core.setTransform(t_ned_angRate, "default_authority");

            t_ned_angRate_lookup = buffer_core.lookupTransform(
              sensorfusion_body_frame, "angrate", msg_stamp);

            odometry_msg.twist.twist.angular.x = t_ned_angRate_lookup.transform.translation.x;
            odometry_msg.twist.twist.angular.y = t_ned_angRate_lookup.transform.translation.y;
            odometry_msg.twist.twist.angular.z = t_ned_angRate_lookup.transform.translation.z;
        } else {
            setNaN(odometry_msg.twist.twist.angular.x);
            setNaN(odometry_msg.twist.twist.angular.y);
            setNaN(odometry_msg.twist.twist.angular.z);
        }
        // std dev currently only available in vel NED frame, from pad solution output
        // disabled covariance output, for velocity published in body frame
        // odometry_msg.twist.covariance[0] = pow(vel_N_std,2);
        // odometry_msg.twist.covariance[7] = pow(vel_E_std,2);
        // odometry_msg.twist.covariance[14] = pow(vel_D_std,2);
        pub_pose_twist_enu.publish(odometry_msg);
    }
}

void IMUUBXData_callback(const sPADFeederMaplab& msg, int imu_id){

    // message time stamp
    ros::Time msg_time;
    if (use_original_timestamps) {
        msg_time = ros::Time((double)msg.imu.towGNSS.towUs / 1000000.0);
    } else {
        msg_time = ros::Time::now();
    }

#ifdef DEBUG_MODE
    cout << "\n*** RECEIVED MESSAGE ***\n";
    cout << "msg.imu.timeInfo_help.timingInfo: " << +msg.imu.timeInfo_help.timingInfo << endl;
    cout << "msg.imu.towGNSS.towUs: " << +msg.imu.towGNSS.towUs << endl;
    cout << "msg.imu.gyro_x.sInt16: " << +msg.imu.gyro_x.sInt16 << endl;
    cout << "msg.imu.gyro_y.sInt16: " << +msg.imu.gyro_y.sInt16 << endl;
    cout << "msg.imu.gyro_z.sInt16: " << +msg.imu.gyro_z.sInt16 << endl;
    cout << "msg.imu.acc_x.sInt16: " << +msg.imu.acc_x.sInt16 << endl;
    cout << "msg.imu.acc_y.sInt16: " << +msg.imu.acc_y.sInt16 << endl;
    cout << "msg.imu.acc_z.sInt16: " << +msg.imu.acc_z.sInt16 << endl;
    cout << "msg.imu.magn_x.sInt16: " << +msg.imu.magn_x.sInt16 << endl;
    cout << "msg.imu.magn_y.sInt16: " << +msg.imu.magn_y.sInt16 << endl;
    cout << "msg.imu.magn_z.sInt16: " << +msg.imu.magn_z.sInt16 << endl;
#endif // DEBUG_MODE

    ImuScalingFactors scale_factors;
    if (imu_id==IMU_STANDARD) {
        scale_factors = scale_factors_imu;
    } else if (imu_id==IMU_OPTIONAL) {
        scale_factors = scale_factors_optimu;
    } else {
        std::cout << "[WARNING] imu_id =" << imu_id << " is not supported." << std::endl;
        return;
    }

    // scale IMU raw data
    double gyro_x = (double)msg.imu.gyro_x.sInt16 * scale_factors.gyro_scale;
    double gyro_y = (double)msg.imu.gyro_y.sInt16 * scale_factors.gyro_scale;
    double gyro_z = (double)msg.imu.gyro_z.sInt16 * scale_factors.gyro_scale;
    double acc_x = (double)msg.imu.acc_x.sInt16 * scale_factors.acc_scale;
    double acc_y = (double)msg.imu.acc_y.sInt16 * scale_factors.acc_scale;
    double acc_z = (double)msg.imu.acc_z.sInt16 * scale_factors.acc_scale;

    // transform IMU raw data to compensate different IMU orientation

    // IMU, ID==2 (see PAD software "extract_ins_data.m")
    /*
        if ins_data.Rcv(i_rcv,1).ID == 2
            delta_phi   = pi;
            delta_theta = 0;
            delta_psi   = pi;
            R_mb = R1(delta_phi)*R2(delta_theta)*R3(delta_psi);
            ins_data.Rcv(i_rcv,1).acc(1:3,1) = R_mb*ins_data.Rcv(i_rcv,1).acc(1:3,1);
            ins_data.Rcv(i_rcv,1).agr(1:3,1) = R_mb*ins_data.Rcv(i_rcv,1).agr(1:3,1);
            ins_data.Rcv(i_rcv,1).mgn(1:3,1) = R_mb*ins_data.Rcv(i_rcv,1).mgn(1:3,1);
        end
        R_mb =

           -1.0000    0.0000         0
            0.0000    1.0000    0.0000
            0.0000    0.0000   -1.0000
    */
    if (imu_id==IMU_OPTIONAL) {
        gyro_x *= -1;
        gyro_z *= -1;
        acc_x *= -1;
        acc_z *= -1;
    }

    if (imu_calibration && calibrMsgCount<calibrMsgTotal){

        // calibrating IMU

        if (calibrMsgCount == 0) {
            cout << "\nCalibrating IMU (average over " << calibrMsgTotal << " measurements)\n";
            cout << "Assumption: Acceleration prior x,y,z (m/s^2): ("
                 << calibrPriorBiasAcc_x << "," << calibrPriorBiasAcc_y << "," << calibrPriorBiasAcc_z << ")" << endl;
        }

        cout << ".";

        sumGyro_x += gyro_x;
        sumGyro_y += gyro_y;
        sumGyro_z += gyro_z;

        sumAcc_x += acc_x;
        sumAcc_y += acc_y;
        sumAcc_z += acc_z;

        ++calibrMsgCount;

        if (calibrMsgCount==calibrMsgTotal) {

            cout << endl;

            //cout << calibrMsgTotal << " measurements complete" << endl;
            cout << "\nIMU calibrated:" << endl;

            biasGyro_x = sumGyro_x / calibrMsgTotal;
            biasGyro_y = sumGyro_y / calibrMsgTotal;
            biasGyro_z = sumGyro_z / calibrMsgTotal;

            biasAcc_x = sumAcc_x / calibrMsgTotal - calibrPriorBiasAcc_x;
            biasAcc_y = sumAcc_y / calibrMsgTotal - calibrPriorBiasAcc_y;
            biasAcc_z = sumAcc_z / calibrMsgTotal - calibrPriorBiasAcc_z;

            cout << cout_indent[0] << "Gyro bias (rad/s):" << endl;
            cout << cout_indent[1] << "x: " << biasGyro_x << endl;
            cout << cout_indent[1] << "y: " << biasGyro_y << endl;
            cout << cout_indent[1] << "z: " << biasGyro_z << endl;

            cout << cout_indent[0] << "Accelerometer bias (m/s^2):" << endl;
            cout << cout_indent[1] << "x: " << biasAcc_x << endl;
            cout << cout_indent[1] << "y: " << biasAcc_y << endl;
            cout << cout_indent[1] << "z: " << biasAcc_z << endl << endl;

            if (pub_imu_calibration) {
                // publish IMU calibration message
                sensor_msgs::Imu bias_calibration_client_msg;
                bias_calibration_client_msg.header.stamp = msg_time;
                bias_calibration_client_msg.header.frame_id = imu_frame;
                bias_calibration_client_msg.linear_acceleration.x = biasAcc_x;
                bias_calibration_client_msg.linear_acceleration.y = biasAcc_y;
                bias_calibration_client_msg.linear_acceleration.z = biasAcc_z;
                bias_calibration_client_msg.angular_velocity.x = biasGyro_x;
                bias_calibration_client_msg.angular_velocity.y = biasGyro_y;
                bias_calibration_client_msg.angular_velocity.z = biasGyro_z;
                pub_imu_calibr.publish(bias_calibration_client_msg);
                cout << "Publish IMU calibration." << endl;
            }

            cout << "Publishing calibrated IMU messages..." << endl;

        }

    } else {

        // IMU calibrated

        angular_velocity_x = gyro_x - biasGyro_x;
        angular_velocity_y = gyro_y - biasGyro_y;
        angular_velocity_z = gyro_z - biasGyro_z;

        // create ROS IMU message
        sensor_msgs::Imu ros_msg;
        ros_msg.header.stamp = msg_time;
        ros_msg.header.frame_id = imu_frame;               // default frame ID for IMUs, see: https://www.ros.org/reps/rep-0145.html
        // orientation - no estimate
        ros_msg.orientation_covariance[0] = -1;     // element 0 set to -1 to indicate no estimate for orientation
        ros_msg.angular_velocity.x = angular_velocity_x;
        ros_msg.angular_velocity.y = angular_velocity_y;
        ros_msg.angular_velocity.z = angular_velocity_z;
        // angular_velocity_covariance - unknown/ initialized with zeros
        ros_msg.linear_acceleration.x = acc_x - biasAcc_x;
        ros_msg.linear_acceleration.y = acc_y - biasAcc_y;
        ros_msg.linear_acceleration.z = acc_z - biasAcc_z;
        // linear_acceleration_covariance - unknown/ initialized with zeros

        // publish IMU message
        pub_imu.publish(ros_msg);

#ifdef DEBUG_MODE
    cout << "\n\n*** PUBLISHED ROS MESSAGE ***\n";
    cout << "ros_msg.header.stamp.sec: " << ros_msg.header.stamp.sec << endl;
    cout << "ros_msg.header.stamp.nsec: " << ros_msg.header.stamp.nsec << endl;
    cout << "ros_msg.angular_velocity.x: " << ros_msg.angular_velocity.x << endl;
    cout << "ros_msg.angular_velocity.y: " << ros_msg.angular_velocity.y << endl;
    cout << "ros_msg.angular_velocity.z: " << ros_msg.angular_velocity.z << endl;
    cout << "ros_msg.linear_acceleration.x: " << ros_msg.linear_acceleration.x << endl;
    cout << "ros_msg.linear_acceleration.y: " << ros_msg.linear_acceleration.y << endl;
    cout << "ros_msg.linear_acceleration.z: " << ros_msg.linear_acceleration.z << endl;
#endif // DEBUG_MODE
    }

}

void WheelOdometryUBXData_callback(const sPADFeederMaplab& msg)
{
    ros::Time msg_time;
    if (use_original_timestamps) {
        msg_time = ros::Time((double)msg.wheel.towGNSS.towUs / 1000000.0);
    } else {
        msg_time = ros::Time::now();
    }

    // velocity in forward direction (in m/s), given
    //  odometer measurements from the left and the right wheel
    const double twist_linear_x = ( (-1)*(1./gear_ratio)*(double)msg.wheel.left_rate.sInt16
                                       + (1./gear_ratio)*(double)msg.wheel.right_rate.sInt16 ) * radius_wheel * M_PI/60.;

    if (pub_odom_twist) {
        // Publish odometry message
        geometry_msgs::TwistStamped wheel_msg;
        wheel_msg.header.stamp = msg_time;
        wheel_msg.header.frame_id = "wheels";
        wheel_msg.twist.linear.x = twist_linear_x;
        wheel_msg.twist.linear.y = 0;
        wheel_msg.twist.linear.z = 0;
        wheel_msg.twist.angular.x = angular_velocity_x;
        wheel_msg.twist.angular.y = angular_velocity_y;
        wheel_msg.twist.angular.z = angular_velocity_z;
        pub_wheel.publish(wheel_msg);

#ifdef DEBUG_MODE
    cout << "\n\n*** PUBLISHED ROS MESSAGE ***\n";
    cout << "wheel_msg.header.stamp.sec: " << wheel_msg.header.stamp.sec << endl;
    cout << "wheel_msg.header.stamp.nsec: " <<wheel_msg.header.stamp.nsec << endl;
    cout << "wheel_msg.twist.linear.x: " << wheel_msg.twist.linear.x << endl;
    cout << "wheel_msg.twist.angular.x: " << wheel_msg.twist.angular.x << endl;
    cout << "wheel_msg.twist.angular.y: " << wheel_msg.twist.angular.y << endl;
    cout << "wheel_msg.twist.angular.z: " << wheel_msg.twist.angular.z << endl;
#endif // DEBUG_MODE
    }

    if (pub_odom_odometry) {
        // Publish odometry message for Cartographer
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg_time;
        odom_msg.header.frame_id = "odom";        // frame_id for pose
        odom_msg.child_frame_id = "wheels";       // frame_id for twist
        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;
        odom_msg.twist.twist.linear.x = twist_linear_x;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = angular_velocity_x;
        odom_msg.twist.twist.angular.y = angular_velocity_y;
        odom_msg.twist.twist.angular.z = angular_velocity_z;
        pub_odom.publish(odom_msg);
    }

}

/*
 * For current ANavS VW Golf test vehicle, and direct CAN interface with DBC file
 * prepared by TU Dresden:
 *  speedFl, speedFr, speedRl, speedRr in km/h
 *  wheelHeading NaN
 *
 * PAD software currently averages rear wheel velocities to compute
 * wheel odometry velocity with origin at back axis mid position.
 */
void CanOdometryUBXData_callback(const sPADFeederMaplab& msg)
{
#ifdef DEBUG_MODE
    cout << "\n\n*** RECEIVED RAW CAN ODOMETRY DATA ***\n";
    cout << "msg.canOdom.towGNSS.towUs: " << +msg.canOdom.towGNSS.towUs << endl;
    cout << "msg.canOdom.speedFl.sFloat: " << double(msg.canOdom.speedFl.sFloat) << endl;
    cout << "msg.canOdom.speedFr.sFloat: " << double(msg.canOdom.speedFr.sFloat) << endl;
    cout << "msg.canOdom.speedRl.sFloat: " << double(msg.canOdom.speedRl.sFloat) << endl;
    cout << "msg.canOdom.speedRr.sFloat: " << double(msg.canOdom.speedRr.sFloat) << endl;
    cout << "msg.canOdom.wheelHeading.sFloat: " << double(msg.canOdom.wheelHeading.sFloat) << endl;
#endif // DEBUG_MODE

    ros::Time msg_time;
    if (use_original_timestamps) {
        msg_time = ros::Time((double)msg.canOdom.towGNSS.towUs / 1000000.0);
    } else {
        msg_time = ros::Time::now();
    }

    if (pub_can_odom_twist) {

        // average velocity of rear wheels
        const double twist_linear_x = 0.5 * double(msg.canOdom.speedRl.sFloat + msg.canOdom.speedRr.sFloat);

        // Publish odometry message
        geometry_msgs::TwistStamped odom_twist_msg;
        odom_twist_msg.header.stamp = msg_time;
        odom_twist_msg.header.frame_id = "odom";
        odom_twist_msg.twist.linear.x = twist_linear_x;
        odom_twist_msg.twist.linear.y = 0;
        odom_twist_msg.twist.linear.z = 0;
        odom_twist_msg.twist.angular.x = 0;
        odom_twist_msg.twist.angular.y = 0;
        odom_twist_msg.twist.angular.z = 0;
        pub_can_odom.publish(odom_twist_msg);

#ifdef DEBUG_MODE
    cout << "\n\n*** PUBLISHED ROS MESSAGE ***\n";
    cout << "odom_twist_msg.header.stamp.sec: " << odom_twist_msg.header.stamp.sec << endl;
    cout << "odom_twist_msg.header.stamp.nsec: " <<odom_twist_msg.header.stamp.nsec << endl;
    cout << "odom_twist_msg.twist.linear.x: " << odom_twist_msg.twist.linear.x << endl;
    cout << "odom_twist_msg.twist.linear.y: " << odom_twist_msg.twist.linear.y << endl;
    cout << "odom_twist_msg.twist.linear.z: " << odom_twist_msg.twist.linear.z << endl;
    cout << "odom_twist_msg.twist.angular.x: " << odom_twist_msg.twist.angular.x << endl;
    cout << "odom_twist_msg.twist.angular.y: " << odom_twist_msg.twist.angular.y << endl;
    cout << "odom_twist_msg.twist.angular.z: " << odom_twist_msg.twist.angular.z << endl;
#endif // DEBUG_MODE
    }

}

/*
 * Decodes UBX header
 *
 *  Extract message ID, or match message ID:
 *      uH.MSGIH == 0 --> message ID is read and stored in uS.sensor, and uS.msgId = 1 is set
 *      uH.MSGIH > 0  --> message ID is read, if it matches uH.MSGIH then uS.msgId = 1 is set, otherwise uS.msgId = 0
 *
 */
bool DeserializeHeader(const unsigned char* buf,  unsigned int nRead, const sUbxHeaderDef& uH,
                       unsigned int& k, sUbxDecodeState& uS)
{
//    cout << "decode header" << endl;
    //
    // Decode header
    //

    while (uS.head1 != 1 && k < nRead) {
      if (buf[k++] == uH.HEAD1) {
        uS.head1 = 1;
//        cout << "HEAD1 found: " << hex << (int) uH.HEAD1 << ", now at k = " << dec << k << endl;
      }
    }

    if (uS.head1 == 1 && uS.head2 != 1 && k < nRead) {
      if (buf[k++] == uH.HEAD2) {
        uS.head2 = 1;
//        cout << "HEAD2 found: " << hex << (int) uH.HEAD2 << ", now at k = " << dec << k << endl;
      } else {
        uS.head1 = 0;
        return false;   //continue;
      }
    }

    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId != 1 && k < nRead) {
      if (buf[k++] == uH.CLASSID) {
        uS.classId = 1;
//        cout << "CLASSID found: " << hex << (int) uH.CLASSID << ", now at k = " << dec << k << endl;
      } else {
        uS.head1 = uS.head2 = 0;
        return false;   //continue;
      }
    }

    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId != 1 && k < nRead) {
      if (uH.MSGID > 0 && buf[k] == uH.MSGID) {
        uS.msgId = 1;
        k++ ;
      }
      else if ( uH.MSGID == 0 ) {
        uS.sensor = buf[k++];
//        cout << "SENSORID found: " << hex << (int) uS.sensor << ", now at k = " << dec << k << endl;
        uS.msgId = 1;
      } else {
        k++;
        return false;
      }
    }

    // Payload length (little endian)
    //    2 byte
    //    4 byte
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
        && uS.payloadData != 1 && uS.bCount2 != uH.bPayloadLen && k < nRead) {

        if (uS.bCount2 >=0 && uS.bCount2 < uH.bPayloadLen) {
            while (k < nRead && uS.bCount2 < uH.bPayloadLen) {
                if (uH.bPayloadLen == 2) {
                    uS.pl2.bytes[(uS.bCount2++)] = buf[k++];
                } else {
                    uS.pl4.bytes[(uS.bCount2++)] = buf[k++];
                }
            }

            if (uS.bCount2 == uH.bPayloadLen) {

                uS.payloadData = 1;
//                cout << "payloadData: " << uS.payloadData << endl;

                if (uH.bPayloadLen == 2) {
                    uS.bPayload = (unsigned int)uS.pl2.uInt16;
//                    cout << "payload length (2 bytes): " << +uS.pl2.uInt16 << ", now at k = " << k << " and bCount2 = " << uS.bCount2 << endl;
                } else {
                    uS.bPayload = uS.pl4.uInt32;
//                    cout << "payload length (4 bytes): " << +uS.pl4.uInt32 << ", now at k = " << k << " and bCount2 = " << uS.bCount2 << endl;
                }
//                cout << "bPayload: " << +uS.bPayload << endl;

            } else {

                return false;   //continue;

            }
        }

    }

    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
            && uS.payloadData == 1) {
        // header decoded completely
        return true;

    } else {
        // header decoding incomplete
        return false;
    }

}

void DeserializeVisUBX2ROS(const SocketData& data, sUbxDecodeState& uS, sMaplabPADGUI& msg,
                            void (*ros_callback)(const sMaplabPADGUI&) )
{

    // Buffer pointing to socket data stream
    const unsigned char* buf = (const unsigned char*)data.getPtrData();
    unsigned int nRead = data.getNumDataChar();

    unsigned int k = 0;      // number of bytes read from buffer

    //
    // Decode header
    //
    sUbxHeaderDef uH;
    uH.HEAD1 = SYNC1_VISION;
    uH.HEAD2 = SYNC2_VISION;
    uH.CLASSID = CLSID_MAPLAB;
    uH.MSGID = MSGID_PADGUI;
    uH.bPayloadLen = 4;

    while (k < nRead)
    {

        if (!DeserializeHeader( buf, nRead, uH,
                                k, uS ) ) {
            continue;
        }

        //
        // Payload / ROS message
        //
        if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
        && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

            if (uS.bCount >=0 && uS.bCount < 4) {
                while (k < nRead && uS.bCount < 4) {
                    char* s = (char* )&(msg.image.header.seq);
                    s[(uS.bCount++)] = buf[k++];
                }

//                if (uS.bCount == 4) {
//                    cout << "msg.image.header.seq: " << msg.image.header.seq << endl;
//                }
            }

            if (uS.bCount >=4 && uS.bCount < 8) {
                while (k < nRead && uS.bCount < 8) {
                    char* s = (char* )&(msg.image.header.stamp.sec);
                    s[(uS.bCount++) - 4] = buf[k++];
                }

//                if (uS.bCount == 8) {
//                    cout << "msg.image.header.stamp.sec: " << msg.image.header.stamp.sec << endl;
//                }
            }

            if (uS.bCount >=8 && uS.bCount < 12) {
                while (k < nRead && uS.bCount < 12) {
                    char* s = (char* )&(msg.image.header.stamp.nsec);
                    s[(uS.bCount++) - 8] = buf[k++];
                }

//                if (uS.bCount == 12) {
//                    cout << "msg.image.header.stamp.nsec: " << msg.image.header.stamp.nsec << endl;
//                }
            }

            if (uS.bCount >=12 && uS.bCount < 16) {
                while (k < nRead && uS.bCount < 16) {
                    char* s = (char* )&(msg.image.height);
                    s[(uS.bCount++) - 12] = buf[k++];
                }

//                if (uS.bCount == 16) {
//                    cout << "msg.image.height: " << msg.image.height << endl;
//                }
            }

            if (uS.bCount >=16 && uS.bCount < 20) {
                while (k < nRead && uS.bCount < 20) {
                    char* s = (char* )&(msg.image.width);
                    s[(uS.bCount++) - 16] = buf[k++];
                }

//                if (uS.bCount == 20) {
//                    cout << "msg.image.width: " << msg.image.width << endl;
//                }
            }

            if (uS.bCount >=20 && uS.bCount < 21) {
                while (k < nRead && uS.bCount < 21) {
                    char* s = (char* )&(msg.image.is_bigendian);
                    s[(uS.bCount++) - 20] = buf[k++];
                }

//                if (uS.bCount == 21) {
//                    cout << "msg.image.is_bigendian: " << msg.image.is_bigendian << endl;
//                }
            }

            if (uS.bCount >=21 && uS.bCount < 25) {
                while (k < nRead && uS.bCount < 25) {
                    char* s = (char* )&(msg.image.step);
                    s[(uS.bCount++) - 21] = buf[k++];
                }

//                if (uS.bCount == 25) {
//                    cout << "msg.image.step: " << msg.image.step << endl;
//                }
            }

            if (uS.bCount >=25 && uS.bCount < uS.bPayload ) {

                //Copy complete memory block at once
                if (k < nRead && uS.bCount < uS.bPayload ) {
                    if (msg.image.data == NULL && uS.bCount == 25) {
                        msg.image.data = new uint8_t[msg.image.height * msg.image.step];
                    }
                    char* s = (char* )(msg.image.data);
                    size_t num = std::min( uS.bPayload - uS.bCount, (unsigned int)(nRead - k));
                    memcpy( &s[uS.bCount - 25], &buf[k], num);
                    k += num;
                    uS.bCount += num;
                }

//                if (uS.bCount == uS.bPayload) {
//                    cout << "msg.image.data: " << " : " << (msg.image.height * msg.image.step) << endl;
//                    for (int i = 0; i < 20; ++i) {
//                        cout << +msg.image.data[i] << " ";
//                    }
//                    cout << endl;
//                }

            }


            //
            //  Payload / ROS message complete
            //
            if (uS.bCount == uS.bPayload)
            {

                //cout << "payload / ROS message complete --> ros_callback()" << endl;

                ros_callback( msg);

                reset(uS);
                continue;

            } // END IF payload complete

        } // END IF decode payload

    }   // END WHILE DECODE

}


void DeserializeVisUBX2ROS(const SocketData& data, sUbxDecodeState& uS, sMaplabPADFeeder& msg,
                            void (*ros_callback)(const sMaplabPADFeeder&) )
{

    // Buffer pointing to socket data stream
    const unsigned char* buf = (const unsigned char*)data.getPtrData();
    unsigned int nRead = data.getNumDataChar();

    unsigned int k = 0;      // number of bytes read from buffer

    //
    // Decode header
    //
    sUbxHeaderDef uH;
    uH.HEAD1 = SYNC1_VISION;
    uH.HEAD2 = SYNC2_VISION;
    uH.CLASSID = CLSID_MAPLAB;
    uH.MSGID = MSGID_PADFEEDER;
    uH.bPayloadLen = 2;

    while (k < nRead)
    {

        if (!DeserializeHeader( buf, nRead, uH,
                                k, uS ) ) {
            continue;
        }

        //
        // Payload / ROS message
        //
        if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
        && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

            if (uS.bCount >=0 && uS.bCount < 4) {
                while (k < nRead && uS.bCount < 4) {
                    char* s = (char* )&(msg.odometryB.header.seq);
                    s[(uS.bCount++)] = buf[k++];
                }

//                if (uS.bCount == 4) {
//                    cout << "msg.odometryB.header.seq: " << msg.odometryB.header.seq << endl;
//                }
            }

            if (uS.bCount >=4 && uS.bCount < 8) {
                while (k < nRead && uS.bCount < 8) {
                    char* s = (char* )&(msg.odometryB.header.stamp.sec);
                    s[(uS.bCount++) - 4] = buf[k++];
                }

//                if (uS.bCount == 8) {
//                    cout << "msg.odometryB.header.stamp.sec: " << msg.odometryB.header.stamp.sec << endl;
//                }
            }

            if (uS.bCount >=8 && uS.bCount < 12) {
                while (k < nRead && uS.bCount < 12) {
                    char* s = (char* )&(msg.odometryB.header.stamp.nsec);
                    s[(uS.bCount++) - 8] = buf[k++];
                }

//                if (uS.bCount == 12) {
//                    cout << "msg.odometryB.header.stamp.nsec: " << msg.odometryB.header.stamp.nsec << endl;
//                }
            }

            if (uS.bCount >=12 && uS.bCount < 20) {
                while (k < nRead && uS.bCount < 20) {
                    char* s = (char* )&(msg.odometryB.twist.twist.linear.x);
                    s[(uS.bCount++) - 12] = buf[k++];
                }

//                if (uS.bCount == 20) {
//                    cout << "msg.odometryB.twist.twist.linear.x: " << msg.odometryB.twist.twist.linear.x << endl;
//                }
            }

            if (uS.bCount >=20 && uS.bCount < 28) {
                while (k < nRead && uS.bCount < 28) {
                    char* s = (char* )&(msg.odometryB.twist.twist.linear.y);
                    s[(uS.bCount++) - 20] = buf[k++];
                }

//                if (uS.bCount == 28) {
//                    cout << "msg.odometryB.twist.twist.linear.y: " << msg.odometryB.twist.twist.linear.y << endl;
//                }
            }

            if (uS.bCount >=28 && uS.bCount < 36) {
                while (k < nRead && uS.bCount < 36) {
                    char* s = (char* )&(msg.odometryB.twist.twist.linear.z);
                    s[(uS.bCount++) - 28] = buf[k++];
                }

//                if (uS.bCount == 36) {
//                    cout << "msg.odometryB.twist.twist.linear.z: " << msg.odometryB.twist.twist.linear.z << endl;
//                }
            }

            if (uS.bCount >=36 && uS.bCount < 44) {
                while (k < nRead && uS.bCount < 44) {
                    char* s = (char* )&(msg.odometryB.twist.twist.angular.x);
                    s[(uS.bCount++) - 36] = buf[k++];
                }

//                if (uS.bCount == 44) {
//                    cout << "msg.odometryB.twist.twist.angular.x: " << msg.odometryB.twist.twist.angular.x << endl;
//                }
            }

            if (uS.bCount >=44 && uS.bCount < 52) {
                while (k < nRead && uS.bCount < 52) {
                    char* s = (char* )&(msg.odometryB.twist.twist.angular.y);
                    s[(uS.bCount++) - 44] = buf[k++];
                }

//                if (uS.bCount == 52) {
//                    cout << "msg.odometryB.twist.twist.angular.y: " << msg.odometryB.twist.twist.angular.y << endl;
//                }
            }

            if (uS.bCount >=52 && uS.bCount < 60) {
                while (k < nRead && uS.bCount < 60) {
                    char* s = (char* )&(msg.odometryB.twist.twist.angular.z);
                    s[(uS.bCount++) - 52] = buf[k++];
                }

//                if (uS.bCount == 60) {
//                    cout << "msg.odometryB.twist.twist.angular.z: " << msg.odometryB.twist.twist.angular.z << endl;
//                }
            }

            // Copy complete memory block at once
            if (uS.bCount >=60 && uS.bCount < 132) {
                if (k < nRead && uS.bCount < 132) {
                    char* s = (char* )&(msg.odometryB.twist.covariance.linear);
                    size_t num = std::min( 132 - uS.bCount, nRead - k);
                    memcpy( &s[uS.bCount - 60], &buf[k], num);
                    k += num;
                    uS.bCount += num;
                }

//                if (uS.bCount == 132) {
//                    cout << "msg.odometryB.twist.covariance.linear: ";
//                    for (int i = 0; i < 9; ++i)
//                        cout << +msg.odometryB.twist.covariance.linear[i] << " ";
//                    cout << endl;
//                }
            }

            // Copy complete memory block at once
            if (uS.bCount >=132 && uS.bCount < 204) {
                if (k < nRead && uS.bCount < 204) {
                    char* s = (char* )&(msg.odometryB.twist.covariance.angular);
                    size_t num = std::min( 204 - uS.bCount, nRead - k);
                    memcpy( &s[uS.bCount - 132], &buf[k], num);
                    k += num;
                    uS.bCount += num;
                }

//                if (uS.bCount == 204) {
//                    cout << "msg.odometryB.twist.covariance.angular: ";
//                    for (int i = 0; i < 9; ++i)
//                        cout << +msg.odometryB.twist.covariance.angular[i] << " ";
//                    cout << endl;
//                }
            }

            if (uS.bCount >=204 && uS.bCount < 212) {
                while (k < nRead && uS.bCount < 212) {
                    char* s = (char* )&(msg.biasAcc.x);
                    s[(uS.bCount++) - 204] = buf[k++];
                }

//                if (uS.bCount == 212) {
//                    cout << "msg.biasAcc.x: " << msg.biasAcc.x << endl;
//                }
            }

            if (uS.bCount >=212 && uS.bCount < 220) {
                while (k < nRead && uS.bCount < 220) {
                    char* s = (char* )&(msg.biasAcc.y);
                    s[(uS.bCount++) - 212] = buf[k++];
                }

//                if (uS.bCount == 220) {
//                    cout << "msg.biasAcc.y: " << msg.biasAcc.y << endl;
//                }
            }

            if (uS.bCount >=220 && uS.bCount < 228) {
                while (k < nRead && uS.bCount < 228) {
                    char* s = (char* )&(msg.biasAcc.z);
                    s[(uS.bCount++) - 220] = buf[k++];
                }

//                if (uS.bCount == 228) {
//                    cout << "msg.biasAcc.z: " << msg.biasAcc.z << endl;
//                }
            }

            if (uS.bCount >=228 && uS.bCount < 236) {
                while (k < nRead && uS.bCount < 236) {
                    char* s = (char* )&(msg.biasGyro.x);
                    s[(uS.bCount++) - 228] = buf[k++];
                }

//                if (uS.bCount == 236) {
//                    cout << "msg.biasGyro.x: " << msg.biasGyro.x << endl;
//                }
            }

            if (uS.bCount >=236 && uS.bCount < 244) {
                while (k < nRead && uS.bCount < 244) {
                    char* s = (char* )&(msg.biasGyro.y);
                    s[(uS.bCount++) - 236] = buf[k++];
                }

//                if (uS.bCount == 244) {
//                    cout << "msg.biasGyro.y: " << msg.biasGyro.y << endl;
//                }
            }

            if (uS.bCount >=244 && uS.bCount < 252) {
                while (k < nRead && uS.bCount < 252) {
                    char* s = (char* )&(msg.biasGyro.z);
                    s[(uS.bCount++) - 244] = buf[k++];
                }

//                if (uS.bCount == 252) {
//                    cout << "msg.biasGyro.z: " << msg.biasGyro.z << endl;
//                }
            }

            if (uS.bCount >=252 && uS.bCount < 260) {
                while (k < nRead && uS.bCount < 260) {
                    char* s = (char* )&(msg.pose.position.x);
                    s[(uS.bCount++) - 252] = buf[k++];
                }

//                if (uS.bCount == 260) {
//                    cout << "msg.pose.position.x: " << msg.pose.position.x << endl;
//                }
            }

            if (uS.bCount >=260 && uS.bCount < 268) {
                while (k < nRead && uS.bCount < 268) {
                    char* s = (char* )&(msg.pose.position.y);
                    s[(uS.bCount++) - 260] = buf[k++];
                }

//                if (uS.bCount == 268) {
//                    cout << "msg.pose.position.y: " << msg.pose.position.y << endl;
//                }
            }

            if (uS.bCount >=268 && uS.bCount < 276) {
                while (k < nRead && uS.bCount < 276) {
                    char* s = (char* )&(msg.pose.position.z);
                    s[(uS.bCount++) - 268] = buf[k++];
                }

//                if (uS.bCount == 272) {
//                    cout << "msg.pose.position.z: " << msg.pose.position.z << endl;
//                }
            }

            if (uS.bCount >=276 && uS.bCount < 284) {
                while (k < nRead && uS.bCount < 284) {
                    char* s = (char* )&(msg.pose.orientation.x);
                    s[(uS.bCount++) - 276] = buf[k++];
                }

//                if (uS.bCount == 284) {
//                    cout << "msg.pose.orientation.x: " << msg.pose.orientation.x << endl;
//                }
            }


            if (uS.bCount >=284 && uS.bCount < 292) {
                while (k < nRead && uS.bCount < 292) {
                    char* s = (char* )&(msg.pose.orientation.y);
                    s[(uS.bCount++) - 284] = buf[k++];
                }

//                if (uS.bCount == 292) {
//                    cout << "msg.pose.orientation.y: " << msg.pose.orientation.y << endl;
//                }
            }

            if (uS.bCount >=292 && uS.bCount < 300) {
                while (k < nRead && uS.bCount < 300) {
                    char* s = (char* )&(msg.pose.orientation.z);
                    s[(uS.bCount++) - 292] = buf[k++];
                }

//                if (uS.bCount == 300) {
//                    cout << "msg.pose.orientation.z: " << msg.pose.orientation.z << endl;
//                }
            }

            if (uS.bCount >=300 && uS.bCount < 308) {
                while (k < nRead && uS.bCount < 308) {
                    char* s = (char* )&(msg.pose.orientation.w);
                    s[(uS.bCount++) - 300] = buf[k++];
                }

//                if (uS.bCount == 308) {
//                    cout << "msg.pose.orientation.w: " << msg.pose.orientation.w << endl;
//                }
            }

            if (uS.bCount >=308 && uS.bCount < 309) {
                while (k < nRead && uS.bCount < 309) {
                    char* s = (char* )&(msg.quality);
                    s[(uS.bCount++) - 308] = buf[k++];
                }

//                if (uS.bCount == 309) {
//                    cout << "msg.quality: " << +msg.quality << endl;
//                }
            }

            //
            //  Payload / ROS message complete
            //
            if (uS.bCount == uS.bPayload)
            {

                //cout << "payload / ROS message complete --> ros_callback()" << endl;

                ros_callback( msg);

                reset(uS);
                continue;

            } // END IF payload complete

        } // END IF decode payload

    }   // END WHILE DECODE

}

void DecodeWheelOdometryUBXData ( const unsigned char* buf, unsigned int nRead, sUbxDecodeState& uS,
                   unsigned int& k, const sUbxHeaderDef& uH, sPADFeederMaplab& msg, void (*ros_callback)(const sPADFeederMaplab&) )
{
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
    && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

        if (uS.bCount >=0 && uS.bCount < 8) {
            while (k < nRead && uS.bCount < 8) {
                uS.wheel.towGNSS.t_bytes[(uS.bCount++)] = buf[k++];
            }
        }
        if (uS.bCount >=8 && uS.bCount <10) {
            while ( k < nRead && uS.bCount < 10) {
                uS.wheel.left_rate.bytes[(uS.bCount++) - 8] = buf[k++];
            }
        }
        if (uS.bCount >= 10 && uS.bCount < 12) {
            while (k < nRead && uS.bCount < 12) {
                uS.wheel.right_rate.bytes[(uS.bCount++) - 10] = buf[k++];
            }
        }
        if (uS.bCount >= 12 && uS.bCount < 14) {
            while (k < nRead && uS.bCount < 14) {
                uS.wheel.left_current.bytes[(uS.bCount++) - 12] = buf[k++];
            }
        }
        if (uS.bCount >= 14 && uS.bCount < 16) {
            while (k < nRead && uS.bCount < 16) {
                uS.wheel.right_current.bytes[(uS.bCount++) - 14] = buf[k++];
            }
        }

        //
        //  Payload / Wheel message complete
        //
        if (uS.bCount == uS.bPayload)
        {

//            cout << "payload / IMU message complete --> ros_callback()" << endl;

            if (overwrite_odo_tow_by_imu_tow) {
                uS.wheel.towGNSS = imu_towGNSS;
            }

            sPADFeederMaplab msg;
            msg.wheel = uS.wheel;

            ros_callback( msg);
            reset(uS);

        } // END IF payload complete

    }
}

void DecodeCanOdometryUBXData ( const unsigned char* buf, unsigned int nRead, sUbxDecodeState& uS,
                   unsigned int& k, const sUbxHeaderDef& uH, sPADFeederMaplab& msg, void (*ros_callback)(const sPADFeederMaplab&) )
{
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
    && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

        uS.canOdom.rcv_id.timingInfo = 0;

        if (uS.bCount >=0 && uS.bCount < 1) {
            while (k < nRead && uS.bCount < 1) {
                uS.canOdom.timingInfo.timingInfoByte[(uS.bCount++)] = buf[k++];
            }
        }
        if (uS.bCount >=1 && uS.bCount < 9) {
            while (k < nRead && uS.bCount < 9) {
                uS.canOdom.towGNSS.t_bytes[(uS.bCount++) - 1] = buf[k++];
            }
        }
        if (uS.bCount >=9 && uS.bCount < 13) {
            while ( k < nRead && uS.bCount < 13) {
                uS.canOdom.speedFl.bytes[(uS.bCount++) - 9] = buf[k++];
            }
        }
        if (uS.bCount >=13 && uS.bCount < 17) {
            while ( k < nRead && uS.bCount < 17) {
                uS.canOdom.speedFr.bytes[(uS.bCount++) - 13] = buf[k++];
            }
        }
        if (uS.bCount >=17 && uS.bCount < 21) {
            while ( k < nRead && uS.bCount < 21) {
                uS.canOdom.speedRl.bytes[(uS.bCount++) - 17] = buf[k++];
            }
        }
        if (uS.bCount >=21 && uS.bCount < 25) {
            while ( k < nRead && uS.bCount < 25) {
                uS.canOdom.speedRr.bytes[(uS.bCount++) - 21] = buf[k++];
            }
        }
        if (uS.bCount >=25 && uS.bCount < 29) {
            while ( k < nRead && uS.bCount < 29) {
                uS.canOdom.wheelHeading.bytes[(uS.bCount++) - 25] = buf[k++];
            }
        }

        // Debug output: Wheel odometry heading information
        // std::cout << std::endl;
        // std::cout << "uS.canOdom.wheelHeading.bytes: " << double(uS.canOdom.wheelHeading.sFloat) << std::endl;

        //
        //  Payload / Wheel message complete
        //
        if (uS.bCount == uS.bPayload)
        {

//            cout << "payload complete --> ros_callback()" << endl;

//            if (overwrite_odo_tow_by_imu_tow) {
//                uS.canOdom.towGNSS = imu_towGNSS;
//            }

            sPADFeederMaplab msg;
            msg.canOdom = uS.canOdom;

            ros_callback( msg);
            reset(uS);

        } // END IF payload complete

    }
}

//  buf, nRead, uS, k, msg, *msg_Maplab, &IMUUBXData_callback
void DecodeIMUUBXData ( const unsigned char* buf,  unsigned int nRead, sUbxDecodeState& uS,
                 unsigned int& k, sUbxHeaderDef& uH, sPADFeederMaplab& msg, void (*ros_callback)(const sPADFeederMaplab&) )
{
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
    && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

//        cout << "\n*** deserialize imu payload ***\n";

        if (uS.bCount >=0 && uS.bCount < 1) {
            while (k < nRead && uS.bCount < 1) {
                uS.imu.timeInfo_help.timingInfoByte[(uS.bCount++)] = buf[k++];
            }
        }

        if (uS.bCount >=1 && uS.bCount < 9) {
            while (k < nRead && uS.bCount < 9) {
                uS.imu.towGNSS.t_bytes[(uS.bCount++) -1] = buf[k++];
            }
        }

        if (uS.bCount >=9 &&  uS.bCount < 11) {
            while (k < nRead && uS.bCount < 11) {
                uS.imu.acc_x.bytes[(uS.bCount++) - 9] = buf[k++];
            }
        }

        if (uS.bCount >= 11 && uS.bCount < 13) {
            while (k < nRead && uS.bCount < 13) {
                uS.imu.acc_y.bytes[(uS.bCount++) - 11] = buf[k++];
            }
        }

        if (uS.bCount >= 13 && uS.bCount < 15) {
            while (k < nRead && uS.bCount < 15) {
                uS.imu.acc_z.bytes[(uS.bCount++) - 13] = buf[k++];
            }
        }

        if (uS.bCount >= 15 && uS.bCount < 17) {
            while (k < nRead && uS.bCount < 17) {
                uS.imu.gyro_x.bytes[(uS.bCount++) - 15] = buf[k++];
            }
        }

        if (uS.bCount >= 17 && uS.bCount < 19) {
            while (k < nRead && uS.bCount < 19) {
                uS.imu.gyro_y.bytes[(uS.bCount++) - 17] = buf[k++];
            }
        }

        if (uS.bCount >= 19 && uS.bCount < 21) {
            while (k < nRead && uS.bCount < 21) {
                uS.imu.gyro_z.bytes[(uS.bCount++) - 19] = buf[k++];
            }
        }

        // magnetometer data
        if (uS.bCount >= 21 && uS.bCount < 23) {
            while (k < nRead && uS.bCount < 23) {
                uS.imu.magn_x.bytes[(uS.bCount++) - 21] = buf[k++];
            }
        }

        if (uS.bCount >= 23 && uS.bCount < 25) {
            while (k < nRead && uS.bCount < 25) {
                uS.imu.magn_y.bytes[(uS.bCount++) - 23] = buf[k++];
            }
        }

        if (uS.bCount >= 25 && uS.bCount < 27) {
            while (k < nRead && uS.bCount < 27) {
                uS.imu.magn_z.bytes[(uS.bCount++) - 25] = buf[k++];
            }
        }

        //
        //  Payload / IMU message complete
        //
        if (uS.bCount == uS.bPayload)
        {

//            cout << "payload / IMU message complete --> ros_callback()" << endl;

            imu_towGNSS = uS.imu.towGNSS;

            sPADFeederMaplab msg;
            msg.imu = uS.imu;

            if (decode_imu_callback)
            {
                ros_callback( msg);
            }

            reset(uS);

        } // END IF payload complete

    } // END IF decode payload
}

//  buf, nRead, uS, k, msg, *msg_Maplab, &IMUUBXData_callback
void DecodeIMUUBXDataAndPublishTime ( const unsigned char* buf,  unsigned int nRead, sUbxDecodeState& uS, unsigned int& k, const sUbxHeaderDef& uH,
                    sPADFeederMaplab& msg, int imu_id, void (*ros_callback)(const sPADFeederMaplab&, int imu_id), const ros::Time& time_before_decode_next)
{
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
    && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

        //cout << "\n*** deserialize imu payload ***\n";

        if (uS.bCount >=0 && uS.bCount < 1) {
            while (k < nRead && uS.bCount < 1) {
                uS.imu.timeInfo_help.timingInfoByte[(uS.bCount++)] = buf[k++];
            }
        }

        if (uS.bCount >=1 && uS.bCount < 9) {
            while (k < nRead && uS.bCount < 9) {
                uS.imu.towGNSS.t_bytes[(uS.bCount++) -1] = buf[k++];
            }
            if (uS.bCount==9) {
                if (mode==MODE_PADSENSORDATA2ROS && pub_time_imu_tow) {
                    // publish time reference message, as soon as imu tow is decoded
                    const ros::Time ros_time_now( ros::Time::now());
                    sensor_msgs::TimeReference msg_tow;
                    msg_tow.header.stamp = ros_time_now;
                    //msg_tow.header.frame_id = '';     // frame_id is not used
                    // pub tow = imu tow + decode time
                    msg_tow.time_ref = ros::Time((double)uS.imu.towGNSS.towUs / 1000000.0)
                                        + (ros_time_now - time_before_decode_next);
                    msg_tow.source = "anavs tow";
                    pub_tow.publish(msg_tow);
                }
            }
        }

        if (uS.bCount >=9 &&  uS.bCount < 11) {
            while (k < nRead && uS.bCount < 11) {
                uS.imu.acc_x.bytes[(uS.bCount++) - 9] = buf[k++];
            }
        }

        if (uS.bCount >= 11 && uS.bCount < 13) {
            while (k < nRead && uS.bCount < 13) {
                uS.imu.acc_y.bytes[(uS.bCount++) - 11] = buf[k++];
            }
        }

        if (uS.bCount >= 13 && uS.bCount < 15) {
            while (k < nRead && uS.bCount < 15) {
                uS.imu.acc_z.bytes[(uS.bCount++) - 13] = buf[k++];
            }
        }

        if (uS.bCount >= 15 && uS.bCount < 17) {
            while (k < nRead && uS.bCount < 17) {
                uS.imu.gyro_x.bytes[(uS.bCount++) - 15] = buf[k++];
            }
        }

        if (uS.bCount >= 17 && uS.bCount < 19) {
            while (k < nRead && uS.bCount < 19) {
                uS.imu.gyro_y.bytes[(uS.bCount++) - 17] = buf[k++];
            }
        }

        if (uS.bCount >= 19 && uS.bCount < 21) {
            while (k < nRead && uS.bCount < 21) {
                uS.imu.gyro_z.bytes[(uS.bCount++) - 19] = buf[k++];
            }
        }

        // magnetometer data
        if (uS.bCount >= 21 && uS.bCount < 23) {
            while (k < nRead && uS.bCount < 23) {
                uS.imu.magn_x.bytes[(uS.bCount++) - 21] = buf[k++];
            }
        }

        if (uS.bCount >= 23 && uS.bCount < 25) {
            while (k < nRead && uS.bCount < 25) {
                uS.imu.magn_y.bytes[(uS.bCount++) - 23] = buf[k++];
            }
        }

        if (uS.bCount >= 25 && uS.bCount < 27) {
            while (k < nRead && uS.bCount < 27) {
                uS.imu.magn_z.bytes[(uS.bCount++) - 25] = buf[k++];
            }
        }

        //
        //  Payload / IMU message complete
        //
        if (uS.bCount == uS.bPayload)
        {

//            cout << "payload / IMU message complete --> ros_callback()" << endl;

            imu_towGNSS = uS.imu.towGNSS;

            sPADFeederMaplab msg;
            msg.imu = uS.imu;

            if (decode_imu_callback)
            {
                ros_callback( msg, imu_id);
            }

//            if (mode==MODE_PADSENSORDATA2ROS) {
//                // publish time reference message
//                const ros::Time ros_time_now( ros::Time::now());
//                sensor_msgs::TimeReference msg_tow;
//                msg_tow.header.stamp = ros_time_now;
//                //msg_tow.header.frame_id = '';     // frame_id is not used
//                msg_tow.time_ref = ros::Time((double)uS.imu.towGNSS.towUs / 1000000.0) + (ros_time_now - time_before_decode_next);
//                msg_tow.source = "anavs tow";
//                pub_tow.publish(msg_tow);
//            }

            reset(uS);

        } // END IF payload complete

    } // END IF decode payload
}

/*
void DecodeSolutionUBXData ( const unsigned char* buf,  unsigned int nRead, sUbxDecodeState& uS,
                 unsigned int& k, const sUbxHeaderDef& uH, sPADSolution& msg, void (*ros_callback)(const sPADSolution&) )
{
    if (uS.head1 == 1 && uS.head2 == 1 && uS.classId == 1 && uS.msgId == 1
    && uS.payloadData == 1 && uS.bCount != uS.bPayload && k < nRead) {

//        cout << "\n*** deserialize sol payload ***\n";

        if (uS.bCount >=0 && uS.bCount < 1) {
            while (k < nRead && uS.bCount < 1) {
                uS.sol.id.timingInfoByte[(uS.bCount++)] = buf[k++];
            }
        }

        if (uS.bCount >=1 && uS.bCount < 3) {
            while (k < nRead && uS.bCount < 3) {
                uS.sol.resCode.bytes[(uS.bCount++) -1] = buf[k++];
            }
        }

        if (uS.bCount >=3 && uS.bCount < 5) {
            while (k < nRead && uS.bCount < 5) {
                uS.sol.week.bytes[(uS.bCount++) -3] = buf[k++];
            }
        }

        if (uS.bCount >=5 && uS.bCount < 13) {
            while (k < nRead && uS.bCount < 13) {
                uS.sol.tow.bytes[(uS.bCount++) -5] = buf[k++];
            }
        }

        if (uS.bCount >=13 && uS.bCount < 15) {
            while (k < nRead && uS.bCount < 15) {
                uS.sol.weekInit.bytes[(uS.bCount++) -13] = buf[k++];
            }
        }

        if (uS.bCount >=15 && uS.bCount < 23) {
            while (k < nRead && uS.bCount < 23) {
                uS.sol.towInit.bytes[(uS.bCount++) -15] = buf[k++];
            }
        }
        // Skip int16 reserved
        if (uS.bCount >=23 && uS.bCount < 25) {
            uS.bCount += 2;
            k += 2;
        }
        // Position NAV-frame
        if (uS.bCount >=25 && uS.bCount < 33) {
            while (k < nRead && uS.bCount < 33) {
                uS.sol.lat.bytes[(uS.bCount++) -25] = buf[k++];
            }
        }

        if (uS.bCount >=33 && uS.bCount < 41) {
            while (k < nRead && uS.bCount < 41) {
                uS.sol.lon.bytes[(uS.bCount++) -33] = buf[k++];
            }
        }

        if (uS.bCount >=41 && uS.bCount < 49) {
            while (k < nRead && uS.bCount < 49) {
                uS.sol.height.bytes[(uS.bCount++) -41] = buf[k++];
            }
        }
        // Position ECEF-frame
        if (uS.bCount >=49 && uS.bCount < 57) {
            while (k < nRead && uS.bCount < 57) {
                uS.sol.x_ECEF.bytes[(uS.bCount++) -49] = buf[k++];
            }
        }

        if (uS.bCount >=57 && uS.bCount < 65) {
            while (k < nRead && uS.bCount < 65) {
                uS.sol.y_ECEF.bytes[(uS.bCount++) -57] = buf[k++];
            }
        }

        if (uS.bCount >=65 && uS.bCount < 73) {
            while (k < nRead && uS.bCount < 73) {
                uS.sol.z_ECEF.bytes[(uS.bCount++) -65] = buf[k++];
            }
        }
        // Basline NED-frame
        if (uS.bCount >=73 && uS.bCount < 81) {
            while (k < nRead && uS.bCount < 81) {
                uS.sol.b_N.bytes[(uS.bCount++) -73] = buf[k++];
            }
        }

        if (uS.bCount >=81 && uS.bCount < 89) {
            while (k < nRead && uS.bCount < 89) {
                uS.sol.b_E.bytes[(uS.bCount++) -81] = buf[k++];
            }
        }

        if (uS.bCount >=89 && uS.bCount < 97) {
            while (k < nRead && uS.bCount < 97) {
                uS.sol.b_D.bytes[(uS.bCount++) -89] = buf[k++];
            }
        }
        // Baseline standard deviation
        if (uS.bCount >=97 && uS.bCount < 105) {
            while (k < nRead && uS.bCount < 105) {
                uS.sol.b_N_std.bytes[(uS.bCount++) -97] = buf[k++];
            }
        }

        if (uS.bCount >=105 && uS.bCount < 113) {
            while (k < nRead && uS.bCount < 113) {
                uS.sol.b_E_std.bytes[(uS.bCount++) -105] = buf[k++];
            }
        }

        if (uS.bCount >=113 && uS.bCount < 121) {
            while (k < nRead && uS.bCount < 121) {
                uS.sol.b_D_std.bytes[(uS.bCount++) -113] = buf[k++];
            }
        }
        // Velocity NED-frame
        if (uS.bCount >=121 && uS.bCount < 129) {
            while (k < nRead && uS.bCount < 129) {
                uS.sol.v_N.bytes[(uS.bCount++) -121] = buf[k++];
            }
        }

        if (uS.bCount >=129 && uS.bCount < 137) {
            while (k < nRead && uS.bCount < 137) {
                uS.sol.v_E.bytes[(uS.bCount++) -129] = buf[k++];
            }
        }

        if (uS.bCount >=137 && uS.bCount < 145) {
            while (k < nRead && uS.bCount < 145) {
                uS.sol.v_D.bytes[(uS.bCount++) -137] = buf[k++];
            }
        }
        // Velocity standard deviation
        if (uS.bCount >=145 && uS.bCount < 153) {
            while (k < nRead && uS.bCount < 153) {
                uS.sol.v_N_std.bytes[(uS.bCount++) -145] = buf[k++];
            }
        }

        if (uS.bCount >=153 && uS.bCount < 161) {
            while (k < nRead && uS.bCount < 161) {
                uS.sol.v_E_std.bytes[(uS.bCount++) -153] = buf[k++];
            }
        }

        if (uS.bCount >=161 && uS.bCount < 169) {
            while (k < nRead && uS.bCount < 169) {
                uS.sol.v_D_std.bytes[(uS.bCount++) -161] = buf[k++];
            }
        }
        // Acceleration in body frame
        if (uS.bCount >=169 && uS.bCount < 177) {
            while (k < nRead && uS.bCount < 177) {
                uS.sol.acc_X.bytes[(uS.bCount++) -169] = buf[k++];
            }
        }

        if (uS.bCount >=177 && uS.bCount < 185) {
            while (k < nRead && uS.bCount < 185) {
                uS.sol.acc_Y.bytes[(uS.bCount++) -177] = buf[k++];
            }
        }

        if (uS.bCount >=185 && uS.bCount < 193) {
            while (k < nRead && uS.bCount < 193) {
                uS.sol.acc_Z.bytes[(uS.bCount++) -185] = buf[k++];
            }
        }
        // Acceleration standard deviation
        if (uS.bCount >=193 && uS.bCount < 201) {
            while (k < nRead && uS.bCount < 201) {
                uS.sol.acc_X_std.bytes[(uS.bCount++) -193] = buf[k++];
            }
        }

        if (uS.bCount >=201 && uS.bCount < 209) {
            while (k < nRead && uS.bCount < 209) {
                uS.sol.acc_Y_std.bytes[(uS.bCount++) -201] = buf[k++];
            }
        }

        if (uS.bCount >=209 && uS.bCount < 217) {
            while (k < nRead && uS.bCount < 217) {
                uS.sol.acc_Z_std.bytes[(uS.bCount++) -209] = buf[k++];
            }
        }
        // Attitude
        if (uS.bCount >=217 && uS.bCount < 225) {
            while (k < nRead && uS.bCount < 225) {
                uS.sol.att_head.bytes[(uS.bCount++) -217] = buf[k++];
            }
        }

        if (uS.bCount >=225 && uS.bCount < 233) {
            while (k < nRead && uS.bCount < 233) {
                uS.sol.att_pitch.bytes[(uS.bCount++) -225] = buf[k++];
            }
        }

        if (uS.bCount >=233 && uS.bCount < 241) {
            while (k < nRead && uS.bCount < 241) {
                uS.sol.att_roll.bytes[(uS.bCount++) -233] = buf[k++];
            }
        }
        // Attitude standard deviation
        if (uS.bCount >=241 && uS.bCount < 249) {
            while (k < nRead && uS.bCount < 249) {
                uS.sol.att_head_std.bytes[(uS.bCount++) -241] = buf[k++];
            }
        }

        if (uS.bCount >=249 && uS.bCount < 257) {
            while (k < nRead && uS.bCount < 257) {
                uS.sol.att_pitch_std.bytes[(uS.bCount++) -249] = buf[k++];
            }
        }

        if (uS.bCount >=257 && uS.bCount < 265) {
            while (k < nRead && uS.bCount < 265) {
                uS.sol.att_roll_std.bytes[(uS.bCount++) -257] = buf[k++];
            }
        }
        // Accuracy
        if (uS.bCount >=265 && uS.bCount < 273) {
            while (k < nRead && uS.bCount < 273) {
                uS.sol.accuracy.bytes[(uS.bCount++) -265] = buf[k++];
            }
        }
        // Skip 1 double reserved
        if (uS.bCount >=273 && uS.bCount < 281) {

            uS.bCount += 8;
            k += 8;
        }
        // Timing Info
        if (uS.bCount >=281 && uS.bCount < 289) {
            while (k < nRead && uS.bCount < 289) {
                uS.sol.timingInfo_Elapsed_GNSS.bytes[(uS.bCount++) -281] = buf[k++];
            }
        }

        if (uS.bCount >=289 && uS.bCount < 297) {
            while (k < nRead && uS.bCount < 297) {
                uS.sol.timingInfo_Elapsed_IMU.bytes[(uS.bCount++) -289] = buf[k++];
            }
        }

        if (uS.bCount >=297 && uS.bCount < 305) {
            while (k < nRead && uS.bCount < 305) {
                uS.sol.timingInfo_Elapsed_BARO.bytes[(uS.bCount++) -297] = buf[k++];
            }
        }

        if (uS.bCount >=305 && uS.bCount < 313) {
            while (k < nRead && uS.bCount < 313) {
                uS.sol.timingInfo_Elapsed_ODO.bytes[(uS.bCount++) -305] = buf[k++];
            }
        }
        // Skip fifth timingInfo double, and 5 double reserved
        if (uS.bCount >=313 && uS.bCount < 361) {
            uS.bCount += 48;
            k += 48;
        }

        // GNSS reception
        if (uS.bCount >=361 && uS.bCount < 369) {
            while (k < nRead && uS.bCount < 369) {
                uS.sol.gnss_rcptn.bytes[(uS.bCount++) -361] = buf[k++];
            }
        }
        // Number of satellites
        if (uS.bCount >=369 && uS.bCount < 370) {
            while (k < nRead && uS.bCount < 370) {
                uS.sol.num_sat.timingInfoByte[(uS.bCount++) -369] = buf[k++];
            }
        }

//        cout << "uS.bCount: " << uS.bCount << endl;

        //
        //  Payload / SOL message complete
        //
        //if (uS.bCount == uS.bPayload)
        if (uS.bCount == 370)
        {

//            cout << "payload / SOL message complete --> ros_callback()" << endl;

            sPADSolution msg;
            msg.sol_ecef = uS.sol;

            ros_callback( msg);

            reset(uS);

        } // END IF payload complete

    }

}
*/

// TODO: Simplify decoding code: Use some basic decoding functions (to solve: handling of cut messages in buffer).
//void DeserializeUBX2ROS(const SocketData& data, sUbxDecodeState& uS, sPADFeederMaplab& msg,
//                            void (*ros_callback)(const sPADFeederMaplab&) )
void DeserializeUBX2ROS(const SocketData& data, sUbxDecodeState& uS)
{

    // IMU detection
    if (!imu_ident_detected) {
        static const ros::Time start_time = ros::Time::now();
        ros::Duration duration = ros::Time::now() - start_time;
        //std::cout << "Detecting IMU, ID=" << imu_ident << " (" << duration.toSec() << "s/" << detect_optimu_timeout_sec << "s)" << std::endl;
        if (duration.toSec() > detect_optimu_timeout_sec) {
            imu_ident_detected = true;
            if (imu_ident > 0) {
                std::cout << "IMU detected (ID=" << imu_ident << ")." << std::endl;
            } else {
                if (mode==MODE_PADSENSORDATA2ROS) {
                    std::cout << "No IMU detected." << std::endl;
                }
            }

        }
    }

    // Define target UBX header
    const sUbxHeaderDef uH = {
        .HEAD1 = SYNC1_UBX,
        .HEAD2 = SYNC2_UBX,
        .CLASSID = CLSID_UBX,
        .MSGID = 0,
        .bPayloadLen = 2};

    const unsigned int kHeaderLen = 4 + uH.bPayloadLen;   // header length in bytes

    // Buffer pointing to socket data stream
    const unsigned char* buf = (const unsigned char*)data.getPtrData();
    const unsigned int nRead = data.getNumDataChar();
    unsigned int k = 0;      // number of bytes read from buffer

    while (k < nRead)
    {

        //cout << "\n*** DeserializeUBX2ROS() ***\n";

        //const double kTimeBeforeDecodeNext = ros::Time::now().toSec();
        const ros::Time kTimeBeforeDecodeNext(ros::Time::now());

        // decode UBX header
        if (!DeserializeHeader(buf,nRead,uH,k,uS)) {
            continue;
        }

//        cout << "\n*** deserialized header ***\n";

        // skip UBX message types that are not relevant for decoding below
        if ( !(uS.sensor==MSGID_IMU || uS.sensor==MSGID_IMUB ||
               uS.sensor==MSGID_WHEEL || uS.sensor==MSGID_CAN ||
               uS.sensor==MSGID_SOL)) {
            //std::cout << "INFO: UBX message type not relevant, skip message (classid="
            //          << hex << (int) uH.CLASSID << ", msgid=" << hex << (int) uS.sensor << ")." << std::endl;
            reset(uS);
            continue;
        }

        // skip checksum verification for solution messages
        if (uS.sensor!=MSGID_SOL) {

            // verify checksum, if message is completely contained in input buffer
            const unsigned int kHeaderStart = k - kHeaderLen;
            const size_t kMsgLen = kHeaderLen + uS.bPayload + 2;
            // Skip message, if it is not contained completely in input buffer, because checksum verification
            //  cannot be performed immediately.
            //  TODO: Handle long messages that are not contained completely in input buffer, currently they
            //      are skipped, because checksum calculation cannot be performed on incomplete data in input buffer.
            //      Observed this case for multi-GNSS messages 02x15 (class, msgid).
            //  EXCEPTION for solution messages, skip
            if (kHeaderStart + uS.bPayload > nRead) {
                std::cout << "[WARNING] Skip message (classid="
                          << hex << (int) uH.CLASSID << ", msgid=" << hex << (int) uS.sensor << ")."
                          << " Checksum cannot be verified." << std::endl;          // message not contained completely in input buffer
                reset(uS);
                continue;
            }
            // verify checksum
            if (!VerifyChecksum((unsigned char*)(buf+kHeaderStart), kMsgLen)) {
                std::cout << "[WARNING] UBX checksum error (classid="
                          << hex << (int) uH.CLASSID << ", msgid=" << hex << (int) uS.sensor << ")." << std::endl;
                reset(uS);
                continue;
            }
            //std::cout << "INFO: UBX checksum verified (classid="
            //          << hex << (int) uH.CLASSID << ", msgid=" << hex << (int) uS.sensor << ")" << std::endl;

        }

        // decode UBX payload
        if (uS.bCount != uS.bPayload && k < nRead) {
//            cout << "uS.sensor: " << +uS.sensor << endl;

            switch (uS.sensor){

                case MSGID_IMU:
                    if (!imu_ident_detected) {
                        imu_ident = (imu_ident > IMU_STANDARD) ? imu_ident : IMU_STANDARD;    // keep imu_id, if imu_id > IMU_STANDARD
                        reset(uS);
                    } else {
                        if (imu_ident==IMU_STANDARD) {
                            //cout << "Decode IMU data (IMU ID=" << imu_ident << ")." << endl;
                            DecodeIMUUBXDataAndPublishTime ( buf, nRead, uS, k, uH, *msg_Maplab, IMU_STANDARD, &IMUUBXData_callback, kTimeBeforeDecodeNext);
                        } else {
                            reset(uS);
                        }
                    }
                    break;

                case MSGID_IMUB:
                    if (!imu_ident_detected && !optimu_disabled) {
                        imu_ident = IMU_OPTIONAL;
                        uS.head1 = uS.head2 = uS.classId = uS.msgId = uS.payloadData = uS.bCount = uS.bCount2 = 0;
                        uS.bPayload = uS.sensor = 0;
                    } else {
                        if (imu_ident==IMU_OPTIONAL) {
                            //cout << "Decode IMU data (IMU ID=" << imu_ident << ")." << endl;
                            DecodeIMUUBXDataAndPublishTime ( buf, nRead, uS, k, uH, *msg_Maplab, IMU_OPTIONAL, &IMUUBXData_callback, kTimeBeforeDecodeNext);
                        } else {
                            reset(uS);
                        }
                    }
                    break;

                case MSGID_WHEEL:
                    if (decode_odometer_data){
                        DecodeWheelOdometryUBXData (buf, nRead, uS, k, uH, *msg_Maplab, &WheelOdometryUBXData_callback);
                        break;
                    } else if (!decode_odometer_data) {
//                        cout << "Do not publish wheel odometry." << endl;
                        k = k + uS.bCount;
                        reset(uS);
                        break;
                    }

                case MSGID_CAN:
                    if (decode_can_odom_data){
                        DecodeCanOdometryUBXData (buf, nRead, uS, k, uH, *msg_Maplab, &CanOdometryUBXData_callback);
                        break;
                    } else {
                        k = k + uS.bCount;
                        reset(uS);
                        break;
                    }

                case MSGID_SOL:
                    std::cout << "[WARNING] DEPRECATED use of 'DeserializeUBX2ROS' to decode ubx solution messages. Skip message. Use 'decode_ubx_solution_publish_ros' instead." << std::endl;
                    // DecodeSolutionUBXData ( buf, nRead, uS, k, uH, *msg_Solution, &SolutionUBXData_callback);
                    k = k + uS.bCount;
                    reset(uS);
                    break;

                default:
#ifdef DEBUG_MODE
                    cout << "Neither IMU, wheel odometry nor solution message decoded." << endl;
#endif
                    reset(uS);
            }

        }

//        if(k == nRead){
//            double time_after_decode = ros::Time::now().toSec();
//            double time_decode = time_after_decode - kTimeBeforeDecodeNext;
//            ros::Time ros_time = ros::Time::now();
//
//            if (mode==MODE_PADSENSORDATA2ROS) {
//                double t_pub_new = ((double)uS.imu.towGNSS.towUs / 1000000.0) + time_decode;
//                geometry_msgs::Pose2D msg_tow;
//                msg_tow.x = t_pub_new;
//                msg_tow.y = ros_time.toSec();
//                pub_tow.publish(msg_tow);
//                if (clock_enabled) {
//                    rosgraph_msgs::Clock msg_clock;
//                    if (use_original_timestamps) {
//                        msg_clock.clock = ros::Time(t_pub_new);
//                    } else {
//                        msg_clock.clock = ros_time;
//                    }
//                    pub_clock.publish(msg_clock);
//                }
//            }
//
//#ifdef DEBUG_MODE
//            if (mode==MODE_PADSOLUTION2ROS) {
//                rosgraph_msgs::Clock msg_clock;
//                msg_clock.clock = ros::Time::now();
//                pub_clock.publish(msg_clock);
//            }
//#endif
////            cout << "t_pub_new = " << t_pub_new << endl;
////            cout << "time_now = " << ros::Time::now().toSec() << endl;
//        }

    }   // END WHILE DECODE

}

void PrintHelp(char **argv, int customer_code = 0)
{
    const std::string progname( GetLastItemFromPath( argv[0]));

    cout << "Mode: padsolution2ros\n" << endl;
    cout << "  Publishes ANavS sensor fusion solution in ROS. Assumes ANavS sensor fusion is running\n";
    cout << "  (eg. on either the ANavS MS-RTK or ISP (remote host)) and TCP/IP data streams are available,\n";
    cout << "  eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 6001). Specify the\n";
    cout << "  remote host's IP for your setup (--ip <host-ip>), and ANavS sensor fusion solution port\n";
    cout << "  if deviating from the default (--port <port>).\n" << endl;
    cout << "  Advertised ROS topics:\n" << endl;
    cout << "     ROS topic                          Description                          ROS message type\n";
    cout << "     ------------------------------------------------------------------------------------------------------------------\n";
    cout << "     /anavs/solution/id                 ID                                   std_msgs/Uint8" << endl;
    cout << "     /anavs/solution/week               Week of current epoch                std_msgs/Uint16" << endl;
    cout << "     /anavs/solution/tow                Time of Week (sec)                   sensor_msgs/TimeReference" << endl;
    cout << "     /anavs/solution/pos                Position (NED* in meters)            geometry_msgs/PointStamped" << endl;
    cout << "     /anavs/solution/pos_llh            Position (lat,lon,height in          geometry_msgs/PointStamped" << endl;
    cout << "                                        deg,deg,meters)" << endl;
    cout << "     /anavs/solution/pos_xyz            Position (ECEF in meters)            geometry_msgs/PointStamped" << endl;
    cout << "     /anavs/solution/att_euler          Attitude/ Euler angles               geometry_msgs/PointStamped" << endl;
    cout << "                                        (heading, pitch, roll in rad)" << endl;
    cout << "     /anavs/solution/att_state          Attitude filter state                std_msgs/UInt8" << endl;
    cout << "     /anavs/solution/rtk_state          RTK filter state                     std_msgs/UInt8" << endl;
    cout << "     /anavs/solution/num_sats           Number of satellites                 std_msgs/UInt8" << endl;
    cout << "     /anavs/solution/imu_calibrated     IMU initialized/calibrated           std_msgs/Bool" << endl;
    cout << "     /anavs/solution/vel                Velocity (NED* in m/s)               geometry_msgs/Vector3Stamped" << endl;
    cout << "     /anavs/solution/acc                Acceleration (body frame in m/s^2)   geometry_msgs/Vector3Stamped" << endl;
    cout << "     /anavs/solution/pos_stddev         Std Dev of position (NED* in meters) geometry_msgs/PointStamped" << endl;
    cout << "     /anavs/solution/vel_stddev         Std Dev of velocity (NED* in m/s)    geometry_msgs/PointStamped" << endl;
    cout << "     /anavs/solution/att_euler_stddev   Std Dev of attitude/                 geometry_msgs/PointStamped" << endl;
    cout << "                                        Euler angles (in rad)" << endl;
    cout << "     /anavs/solution/accuracy           Estimated accuracy (m)               geometry_msgs/PointStamped" << endl;
    cout << "                                        (stored in: point.x)" << endl;
    cout << "     ------------------------------------------------------------------------------------------------------------------\n";
    cout << "     Optional:" << endl;
    cout << "     /anavs/solution/pose_enu           Pose (ENU in meters)                 geometry_msgs/PoseWithCovarianceStamped" << endl;
    cout << "     /anavs/solution/odom               Odometry (Pose: ENU in meters/       nav_msgs/Odometry" << endl;
    cout << "                                        Twist: body frame, velocity in m/s" << endl;
    cout << "                                        angular rate in rad/s)" << endl;
    cout << "     /anavs/solution/ang_rate           Angular rate (NED* in rad/s)         geometry_msgs/Vector3Stamped" <<endl;
    cout << "     /path                              Path for visualization               nav_msgs/Path" << endl;
    cout << "     ------------------------------------------------------------------------------------------------------------------\n";
    cout << "     *): The default frame for 'pos', 'vel' and 'ang_rate' is NED, but it may vary if a specific customer code\n";
    cout << "         is used (see ANAVS Wizard: 'Customer-Code', and ANAVS.conf: 'customer_code').\n";
    cout << "         Also the body frame definition may vary if a specific customer code is used.\n";
    cout << "     ------------------------------------------------------------------------------------------------------------------\n";
    cout << "     The standard ROS topics provided correspond to the fields of the Standard Binary Solution Message\n";
    cout << "     documented in the User Reference Guide, section 6.1.:\n";
    cout << "     https://anavs.com/knowledgebase/user-reference-guide/\n";
    cout << "     In the ROS messages angular measurements are provided in 'rad' rather than 'deg'.\n";
    cout << endl;

    if (customer_code!=1) {
        cout << "Mode: padsensordata2ros\n" << endl;
        cout << "  Publishes ANavS sensor data in ROS. Assumes powered on ANavS MS-RTK or ISP (remote host)\n";
        cout << "  (ANavS sensor fusion does not need to be running) and TCP/IP data streams are available,\n";
        cout << "  eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 4001). Specify the\n";
        cout << "  remote host's IP for your setup (--ip <host-ip>).\n" << endl;
        cout << "  Advertised ROS topics:\n" << endl;
        cout << "     ROS topic                        Description                   ROS message type\n";
        cout << "     -------------------------------------------------------------------------------------------------------\n";
        cout << "     /imu0                            IMU messages                  sensor_msgs/Imu" << endl;
        cout << endl;
    }

    if (customer_code!=1 && customer_code!=5) {
        cout << "Mode: leapseconds2ros\n" << endl;
        cout << "  Publishes leapseconds from GPS in ROS. Assumes powered on ANavS MS-RTK or ISP (remote host)\n";
        cout << "  (ANavS sensor fusion does not need to be running) and TCP/IP data streams are available,\n";
        cout << "  eg. via Ethernet connection (default settings IP: 127.0.0.1, port: 4004). Specify the\n";
        cout << "  remote host's IP for your setup (--ip <host-ip>).\n" << endl;
        cout << "  Advertised ROS topics:\n" << endl;
        cout << "     ROS topic                        Description                   ROS message type\n";
        cout << "     -------------------------------------------------------------------------------------------------------\n";
        cout << "     /leapseconds                     leap seconds from             std_msgs/Int8" << endl;
        cout << "                                      GPS receiver (second)" << endl;
        cout << endl;
    }

    cout << "Usage:\n\n";
    cout << "  " << progname << " padsolution2ros [--ip <host-ip>] [--port <port>] [options]\n";
    if (customer_code!=1) {
        cout << "  " << progname << " padsensordata2ros [--ip <host-ip>] [--port <port>] [options]\n";
    }
    if (customer_code!=1 && customer_code!=5) {
        cout << "  " << progname << " leapseconds2ros [--ip <host-ip>] [--port <port>]\n";
    }
    if (customer_code==0) {
        cout << "  " << progname << " debug_padfeeder_vio [--ip <host-ip>] [--port <port>]\n";
        cout << "  " << progname << " debug_padgui_image [--ip <host-ip>] [--port <port>]\n";
    }

    cout << "\nOptions:\n";
    cout << endl;

    cout << "  -h --help\t\t\t\t" << "Show help.\n";
    cout << "  --ip <host-ip>\t\t\t" << "IP address of remote host [default: 127.0.0.1].\n";
    cout << "  --port <host-port>\t\t\t" << "TCP/IP data streams port. ANavS sensor fusion solution [default: 6001], sensor data [default: 4001].\n";

    if (customer_code==0) {
        cout << "  --reconnection_delay <sec>\t\t" << "Specify TCP client reconnection delay in seconds [default: 5]\n";
    }
    cout << endl;

    cout << "  padsolution2ros:" << endl;
    cout << "  --topic_prefix <topic-prefix>\t\t" << "Prefix for all ROS topics published [default: /anavs/solution]." << endl;
    cout << "  --use_original_timestamps\t\t" << "Use original ANavS solution data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].\n";
    cout << "  --enable_odom\t\t\t\t" << "Enables publishing odometry (topic: /odom; type: nav_msgs::Odometry) and angular rate (topic: /ang_rate; type: geometry_msgs/Vector3Stamped)." << endl;
    cout << "  --enable_pose\t\t\t\t" << "Enables publishing pose (topic: /pose_enu; type: geometry_msgs::PoseWithCovarianceStamped)." << endl;
    cout << "  --odom_only\t\t\t\t" << "Publish only the odometry message '/odom'." << endl;
    cout << "  --pose_only\t\t\t\t" << "Publish only the pose message '/pose_enu'." << endl;
    cout << "  --publisher_queue_size <queue-size>\t" << "Sets ROS publisher queue sizes for all messages published. [default: 1]" << endl;
    cout << "  --pose_enu_frame <frame-id>\t\t" << "Specifies frame_id of published pose (topic: /pose_enu)' [default: map]." << endl;
    cout << "  --remove_pos_offset\t\t\t" << "Enables removing initial position offset for publishing pose 'pose_enu', and odometry 'odom'." << endl;
    cout << "  --publish_to_tf [--published_frame\t" << "Enables publishing transformation to ROS tf. Optional: Specify child_frame_id [default: base_link]." << endl;
    cout << "       <child-frame-id>]\t" << endl;
    cout << "  --publish_path\t\t\t" << "Enables publishing path (topic: /path; type: nav_msgs::Path) created from poses (topic: /pose_enu)." << endl;
    cout << endl;

    if (customer_code!=1) {
        cout << "  padsensordata2ros:" << endl;
        cout << "  --topic_prefix <topic-prefix>\t\t" << "Prefix for all ROS topics published [default: no prefix]." << endl;
        cout << "  --use_original_timestamps\t\t" << "Use original ANavS sensor data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].\n";
        cout << "  --publisher_queue_size <queue-size>\t" << "Sets ROS publisher queue sizes for all messages published. [default: 1]" << endl;
        // IMU
        cout << "  --imu_topic [<topic-name>]\t\t" << "Specifies IMU topic name [default: /imu0].\n";
        cout << "  --calibrate_imu\t\t\t" << "Enables IMU calibration.\n";
        cout << "  --pub_imu_calibration [<topic-name>]\t" << "Enables publishing IMU calibration. Optional: Specify topic name [default: /imu_calibration].\n";
        if (customer_code==0) {
            // internal only
            cout << "  --pub_time_imu_tow [<topic-name>]\t" << "Enables publishing IMU time (GPS tow). Optional: Specify topic name [default: /time_imu_tow].\n";

            // odometry
            cout << "  --pub_odom_twist [<topic-name>]\t" << "Enables decoding ANavS odometer raw data, and publishing odometry messages (geometry_msgs::TwistStamped). Optional: Specify topic name [default: /abss/twist]\n";
            cout << "  --pub_odom_odometry [<topic-name>]\t" << "Enables decoding ANavS odometer raw data, and publishing odometry messages (nav_msgs::Odometry). Optional: Specify topic name [default: /odo]\n";

            // CAN odometry
            cout << "  --pub_can_odom_twist [<topic-name>]\t" << "Enables decoding ANavS CAN odometry data, and publishing odometry messages (geometry_msgs::TwistStamped). Optional: Specify topic name [default: /abss/twist]\n";

            // for ROVIOLI input: odometry via geometry_msgs::TwistStamped
            // for Google Cartographer: odometry via nav_msgs::Odometry
        }
        if (customer_code==0) {
            // internal only
            cout << "  --overwrite_odo_tow_by_imu_tow\t" << "Overwrites ANavS odometer raw data timestamp by IMU raw data timestamp (GNSS time-of-weeks).\n";
            cout << "  --disable_imu_callback\t\t" << "Disables IMU callback function. Use to reduce load when only publishing time topic.\n";
            cout << "  --disable_optimu\t\t\t" << "Disables optional IMU, standard IMU is used if available.\n";
            cout << "  --bgx <imu-gyro-bias-x> --bgy <imu-gyro-bias-y> --bgz <imu-gyro-bias-z> --bax <imu-acc-bias-x> --bay <imu-acc-bias-x> --baz <imu-acc-bias-x>\t" << "Manually define IMU gyroscope and accelerometer biases\n"
                 << "                                        (only applies, if IMU calibration is disabled)\n";
        }
        cout << endl;
    }

    if (customer_code!=1 && customer_code!=5) {
        cout << "  leapseconds2ros:" << endl;
        cout << "  --use_original_timestamps\t\t" << "Use original ANavS sensor data timestamps (GPS tow) as ROS message timestamp [default: local host ROS time].\n";
        cout << "  --publisher_queue_size <queue-size>\t" << "Sets ROS publisher queue sizes for all messages published. [default: 1]" << endl;
        cout << endl;
    }

    cout << endl;
}

void CleanUp()
{
    delete msg_PADFeeder;
    msg_PADFeeder = NULL;
    if (msg_PADGUI != NULL) {
        delete msg_PADGUI->image.data;
        msg_PADGUI->image.data = NULL;
    }
    delete msg_PADGUI;
    msg_PADGUI = NULL;
    delete msg_Maplab;
    msg_Maplab = NULL;
    delete msg_Solution;
    msg_Solution = NULL;
}

static int s_interrupted = 0;
static void s_signal_handler (int signal_value)
{
    cout << "\n \n************************************ \n";
    cout << "Cleaning up and exiting program ... \n";
    cout << "************************************\n";

    s_interrupted = 1;

    CleanUp();

#ifdef DEBUG_MODE
    //outfile.close();
#endif

    fflush(stdout);
    exit(0);
}

static void s_catch_signals (void)
{
    struct sigaction action;
    action.sa_handler = s_signal_handler;
    action.sa_flags = 0;
    sigemptyset (&action.sa_mask);
    sigaction (SIGINT, &action, NULL);
    sigaction (SIGTERM, &action, NULL);
}

void decode_ubx_leapseconds_publish_ros(shared_ptr<StreamQuantum> quantum)
{
    uint8_t cls = quantum->getNumericAttribute("cls");
    uint8_t id = quantum->getNumericAttribute("id");

    if (cls == UBX_RXM_RAWX_CLS && id == UBX_RXM_RAWX_ID) {

        //std::cout << "Raw UBX package found" << std::endl;

        UbxRawx rp(quantum);

        std_msgs::Int8 msg_leapseconds;
        msg_leapseconds.data = int(rp.leapS);
        //std::cout << int(rp.leapS) << std::endl;

        pub_leapseconds.publish(msg_leapseconds);

    }

}

void decode_ubx_solution_publish_ros(shared_ptr<StreamQuantum> quantum)
{
    uint8_t cls = quantum->getNumericAttribute("cls");
    uint8_t id = quantum->getNumericAttribute("id");

    if (cls == UBX_SOLUTION_PKG_CLS && id == UBX_SOLUTION_PKG_ID)
    {
        // std::cout << "pad solution package found" << std::endl;

        // get pad solution package
        UbxPadSol sp(quantum, decode_filters);

        SolutionUBXData_callback(sp);

        /*
        GnggaPackage gngga_pkg;
        gngga_pkg.pack();
        std::string gngga_pkg_str = gngga_pkg.getStringRepresentation();
        */

        /*
        // create NMEA GTPOS message
        GtposPackage gtpos_msg;
        // common fields
        gtpos_msg.gps_week_nr = sp.week;
        gtpos_msg.gps_tow = sp.tow;
        gtpos_msg.src_id = 1;
        gtpos_msg.health_status = 1.0;
        gtpos_msg.sp_lh = 0.0;
        strcpy(gtpos_msg.sp_det_talker_id, "");
        gtpos_msg.sp_det_src_id = 0;
        // POS fields
        gtpos_msg.pos_x_ECEF = sp.ECEF_X;
        gtpos_msg.pos_y_ECEF = sp.ECEF_Y;
        gtpos_msg.pos_z_ECEF = sp.ECEF_Z;
        gtpos_msg.sigma_pos_x_ECEF = sp.bStdDev[0];   // Todo: convert stddev from NED to ECEF
        gtpos_msg.sigma_pos_y_ECEF = sp.bStdDev[1];
        gtpos_msg.sigma_pos_z_ECEF = sp.bStdDev[2];

        // create NMEA GTPOS message
        GtvelPackage gtvel_msg;
        // common fields
        gtvel_msg.gps_week_nr = sp.week;
        gtvel_msg.gps_tow = sp.tow;
        gtvel_msg.src_id = 1;
        gtvel_msg.health_status = 1.0;
        gtvel_msg.sp_lh = 0.0;
        strcpy(gtvel_msg.sp_det_talker_id, "");
        gtvel_msg.sp_det_src_id = 0;
        // VEL fields
        gtvel_msg.vel_lin_x_body = sp.vel[0];   // Todo: convert from NED to body
        gtvel_msg.vel_lin_y_body = sp.vel[1];
        gtvel_msg.vel_lin_z_body = sp.vel[2];
        gtvel_msg.vel_ang_x_body = 0.0;         // Todo: get from pad solution (from filters: acc)?
        gtvel_msg.vel_ang_y_body = 0.0;
        gtvel_msg.vel_ang_z_body = 0.0;
        gtvel_msg.sigma_vel_lin_x_body = sp.velStdDev[0];   // Todo: convert from NED to body
        gtvel_msg.sigma_vel_lin_y_body = sp.velStdDev[1];
        gtvel_msg.sigma_vel_lin_z_body = sp.velStdDev[2];
        gtvel_msg.sigma_vel_ang_x_body = 0.0;   // Todo: get from pad solution (from filters: acc)?
        gtvel_msg.sigma_vel_ang_y_body = 0.0;
        gtvel_msg.sigma_vel_ang_z_body = 0.0;

        // get NMEA strings
        std::string gtpos_msg_str = gtpos_msg.getStringRepresentation();
        std::string gtvel_msg_str = gtvel_msg.getStringRepresentation();

        // debug
        std::cout << gtpos_msg_str;
        std::cout << gtvel_msg_str;

        // write NMEA messages to file
        nmea_outfile << gtpos_msg_str.c_str();
        nmea_outfile << gtvel_msg_str.c_str();
        */

    }
}

int main(int argc, char **argv)
{

    cout << "\n***********************************************************\n";
    cout << "* ANavS ROS-Ethernet Adapter (TCP-Client: v" << VER_ROSETHCLI << "[" << customer_code << "])     *\n";
    cout << "***********************************************************\n\n";

#ifdef DEBUG_MODE
    cout << "Debug mode: ON\n" << endl;
#endif // DEBUG_MODE

    /* Handling Ctrl-C cleanly
    reference: http://zguide.zeromq.org/cpp:interrupt */
    s_catch_signals ();

    if (argc==2 && (CmdOptionExists(argv, argv+argc, "-h") || CmdOptionExists(argv, argv+argc, "--help"))) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    if ((argc < 2)) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    // TEST_eulerNED2quatENU();

    //
    // Client modes
    //
    int cidx = 1;   // cmdline argument index
    const string mode_str(argv[cidx++]);

    if (!strcmp(mode_str.c_str(), "debug_padfeeder_vio") && argc>=2) {
        mode = MODE_DEBUG_PADFEEDER_VIO;
    } else if (!strcmp(mode_str.c_str(), "debug_padgui_image") && argc>=2) {
        mode = MODE_DEBUG_PADGUI_IMAGE;
    } else if (!strcmp(mode_str.c_str(), "padsensordata2ros") && argc>=2) {
        mode = MODE_PADSENSORDATA2ROS;
    } else if (!strcmp(mode_str.c_str(), "padsolution2ros") && argc>=2) {
        mode = MODE_PADSOLUTION2ROS;
    } else if (!strcmp(mode_str.c_str(), "leapseconds2ros") && argc>=2) {
        mode = MODE_LEAPSECONDS2ROS;
    } else {
        mode = MODE_UNDEFINED;
    }

    if (mode==MODE_UNDEFINED) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if ((customer_code==1) &&
            mode!=MODE_PADSOLUTION2ROS) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if ((customer_code==5) &&
            (mode!=MODE_PADSOLUTION2ROS && mode!=MODE_PADSENSORDATA2ROS)) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if (!(customer_code==0) &&
            (mode==MODE_DEBUG_PADFEEDER_VIO || mode==MODE_DEBUG_PADGUI_IMAGE)) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    string ipaddr("127.0.0.1");
    int port = 6001;

    if (mode == MODE_PADSENSORDATA2ROS) {
        port = 4001;
    }
    if (mode == MODE_LEAPSECONDS2ROS) {
        port = 4004;
    }

    if (CmdOptionExists(argv, argv+argc, "--ip")) {
        GetCmdOption(argv, argv+argc, "--ip", ipaddr);
    }
    if (CmdOptionExists(argv, argv+argc, "--port")) {
        GetCmdOption(argv, argv+argc, "--port", port);
    }

    cout << "Mode: " << mode_str << "\n" << endl;
    cout << "ANavS sensor fusion remote host settings:" << endl;
    cout << cout_indent[0] << "IP: " << ipaddr << endl;
    cout << cout_indent[0] << "port: " << port << "\n" << endl;

    if (mode==MODE_PADSOLUTION2ROS) {
        // Read ANavS config file
        cout << "Reading file '" << anavs_config_filename << "'..." << endl;

         // specify parameters to read
        std::vector< std::string > param_names;
        param_names.push_back("customer_code");

        std::vector<double> param_values;
        param_values.push_back(pad_customer_code);

        if (!ReadAnavsConfigFile(anavs_config_filename, param_names, param_values)) {
            cout << cout_indent[0] << "\tFile cannot be opened: " << anavs_config_filename << ". Apply standard parameters." << endl;
        } else {
           int idx = 0;
           pad_customer_code = static_cast<int>( param_values[idx++]);
        }

        cout << cout_indent[0] << "customer_code: " << pad_customer_code << endl;
        cout << endl;

        if (pad_customer_code==0) {
            cout << "[INFO] The default coordinate frame definition applies:" << endl;
        } else {
            cout << "[INFO] A customer specific coordinate frame definition may apply deviating from the default." << endl;
            // disable topics odom and pose
            // conversions currently rely on pos and vel given in NED frame
            // and Euler angles given in default body frame.
            cout << "The default coordinate frame definition is the following:" << endl;
        }
        cout << "NED frame applies to topics: 'pos', 'pos_stddev', 'vel' and 'vel_stddev'." << endl;
        cout << "The body frame definition is the following:" << endl;
        cout << "- x-axis points forward" << endl;
        cout << "- y-axis points to the right" << endl;
        cout << "- z-axis points downward\n" << endl;

    }

    if (mode==MODE_PADSENSORDATA2ROS) {
        // Read ANavS config file
        cout << "Reading file '" << anavs_config_filename << "'..." << endl;

         // specify parameters to read
        std::vector< std::string > param_names;
        param_names.push_back("PAD-gear_ratio");
        param_names.push_back("PAD-wheel_radius");

        std::vector<double> param_values;
        param_values.push_back(gear_ratio);
        param_values.push_back(radius_wheel);
        //param_names.push_back("PAD-wheel_distance");   // not used atm

        if (!ReadAnavsConfigFile(anavs_config_filename, param_names, param_values)) {
            cout << cout_indent[0] << "\tFile cannot be opened: " << anavs_config_filename << ". Apply standard parameters." << endl;
        } else {
           int idx = 0;
           gear_ratio = param_values[idx++];
           radius_wheel = param_values[idx++];
        }

        cout << cout_indent[0] << "PAD-gear_ratio: " << gear_ratio << endl;
        cout << cout_indent[0] << "PAD-wheel_radius: " << radius_wheel << endl;
        cout << endl;
    }

    vector<string> rostopics_str;   // used for MODE_PADSOLUTION2ROS

    // TODO: Also use ros topic string vector for other modes, esp. MODE_PADSENSORDATA2ROS?
    string imu_topic("/imu0");
    string time_topic("/time_imu_tow");
    string imu_calibr_topic("/imu_calibration");
    string odom_twist_topic("/abss/twist");
    string odom_odometry_topic("/odo");
    string can_odom_twist_topic("/abss/twist");
    string leapseconds_topic("/leapseconds");
    //string clock_topic("/clock");

    if (mode == MODE_PADSENSORDATA2ROS)
    {
        cout << "Options set:" << endl;
        // get cmdline options
        string topic_prefix("");
        if (CmdOptionExists(argv, argv+argc, "--topic_prefix")) {
            GetCmdOption(argv, argv+argc, "--topic_prefix", topic_prefix);
            cout << cout_indent[0] << "topic_prefix: " << topic_prefix << endl;
        }
        if (CmdOptionExists(argv, argv+argc, "--use_original_timestamps")) {
            use_original_timestamps = true;
            cout << cout_indent[0] << "use_original_timestamps" << endl;
        }
        if (CmdOptionExists(argv, argv+argc, "--imu_topic")) {
            GetCmdOption(argv, argv+argc, "--imu_topic", imu_topic);
        }
        imu_calibration = CmdOptionExists(argv, argv+argc, "--calibrate_imu");
        if (imu_calibration) {
            cout << cout_indent[0] << "IMU calibration" << endl;
        }
        if (CmdOptionExists(argv, argv+argc, "--pub_imu_calibration")) {
            pub_imu_calibration = true;
            GetCmdOption(argv, argv+argc, "--pub_imu_calibration", imu_calibr_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--publisher_queue_size")) {
            GetCmdOption(argv, argv+argc, "--publisher_queue_size", kPublisherQueueSize);
            cout << cout_indent[0] << "publisher_queue_size: " << kPublisherQueueSize << endl;
        }

        if (customer_code==0) {
            if (CmdOptionExists(argv, argv+argc, "--pub_time_imu_tow")) {
                pub_time_imu_tow = true;
                GetCmdOption(argv, argv+argc, "--pub_time_imu_tow", time_topic);
            }
            if (CmdOptionExists(argv, argv+argc, "--pub_odom_twist")) {
                decode_odometer_data = true;
                cout << cout_indent[0] << "ANavS odometer raw data decoding" << endl;
                pub_odom_twist = true;
                GetCmdOption(argv, argv+argc, "--pub_odom_twist", odom_twist_topic);
            }
            if (CmdOptionExists(argv, argv+argc, "--pub_odom_odometry")) {
                decode_odometer_data = true;
                cout << cout_indent[0] << "ANavS odometer raw data decoding" << endl;
                pub_odom_odometry = true;
                GetCmdOption(argv, argv+argc, "--pub_odom_odometry", odom_odometry_topic);
            }
            if (CmdOptionExists(argv, argv+argc, "--pub_can_odom_twist")) {
                decode_can_odom_data = true;
                cout << cout_indent[0] << "ANavS CAN odometry raw data decoding" << endl;
                pub_can_odom_twist = true;
                GetCmdOption(argv, argv+argc, "--pub_can_odom_twist", can_odom_twist_topic);
            }
        }

//        if (CmdOptionExists(argv, argv+argc, "--clock")) {
//            clock_enabled = true;
//            cout << cout_indent[0] << "[On] Topic " << clock_topic << " enabled, publish time messages." << endl;
//        }

        bool all_biases_given = false;

        if (customer_code==0) {
            overwrite_odo_tow_by_imu_tow = CmdOptionExists(argv, argv+argc, "--overwrite_odo_tow_by_imu_tow");
            if (overwrite_odo_tow_by_imu_tow) {
                cout << cout_indent[0] << "Overwrite ANavS odometer raw data timestamps by IMU timestamps." << endl;
            }
            decode_imu_callback = !CmdOptionExists(argv, argv+argc, "--disable_imu_callback");
            if (decode_imu_callback) {
                cout << cout_indent[0] << "Disabled IMU callback function." << endl;
            }
            optimu_disabled = CmdOptionExists(argv, argv+argc, "--disable_optimu");
            if (optimu_disabled) {
                cout << cout_indent[0] << "Disabled optional IMU." << endl;
            }
            all_biases_given = CmdOptionExists(argv, argv+argc, "--bgx") &&
                              CmdOptionExists(argv, argv+argc, "--bgy") &&
                              CmdOptionExists(argv, argv+argc, "--bgz") &&
                              CmdOptionExists(argv, argv+argc, "--bax") &&
                              CmdOptionExists(argv, argv+argc, "--bay") &&
                              CmdOptionExists(argv, argv+argc, "--baz");

            if (!imu_calibration && all_biases_given) {
                GetCmdOption(argv, argv+argc, "--bgx", biasGyro_x);
                GetCmdOption(argv, argv+argc, "--bgy", biasGyro_y);
                GetCmdOption(argv, argv+argc, "--bgz", biasGyro_z);
                GetCmdOption(argv, argv+argc, "--bax", biasAcc_x);
                GetCmdOption(argv, argv+argc, "--bay", biasAcc_y);
                GetCmdOption(argv, argv+argc, "--baz", biasAcc_z);

                cout << "\nGyro biases:" << endl;
                cout << cout_indent[0] << "biasGyro_x: " << biasGyro_x << endl;
                cout << cout_indent[0] << "biasGyro_y: " << biasGyro_y << endl;
                cout << cout_indent[0] << "biasGyro_z: " << biasGyro_z << endl;
                cout << "\nAccelerometer biases:" << endl;
                cout << cout_indent[0] << "biasAcc_x: " << biasAcc_x << endl;
                cout << cout_indent[0] << "biasAcc_y: " << biasAcc_y << endl;
                cout << cout_indent[0] << "biasAcc_z: " << biasAcc_z << endl;
            }
        }
        cout << endl;

        // advertise topics
        std::vector<std::string> rostopic_anavs_str;
        rostopic_anavs_str.push_back(imu_topic);
        if (pub_imu_calibration) {
            rostopic_anavs_str.push_back(imu_calibr_topic);
        }
        // customer_coder==0
        if (pub_time_imu_tow) {
            rostopic_anavs_str.push_back(time_topic);
        }
        if (pub_odom_twist) {
            rostopic_anavs_str.push_back(odom_twist_topic);
        }
        if (pub_odom_odometry) {
            rostopic_anavs_str.push_back(odom_odometry_topic);
        }
        if (pub_can_odom_twist) {
            rostopic_anavs_str.push_back(can_odom_twist_topic);
        }
//        if (clock_enabled) {
//            rostopic_anavs_str.push_back(clock_topic);
//        }

        // add topic suffix
        for(std::vector<std::string>::size_type i=0; i < rostopic_anavs_str.size(); i++){
            rostopics_str.push_back(topic_prefix);
            rostopics_str[i] += rostopic_anavs_str[i];
        }

        cout << "Advertise ROS topics: " << "\n";
        for (std::vector<std::string>::size_type i = 0; i < rostopics_str.size(); ++i) {
            cout << cout_indent[0] << rostopics_str[i] << "\n";
        }
        cout << endl;

    } else if (mode == MODE_PADSOLUTION2ROS) {

        cout << "Options set: " << "\n";
        bool any_option_set = false;
        string topic_prefix("/anavs/solution");
        if (CmdOptionExists(argv, argv+argc, "--topic_prefix")) {
            GetCmdOption(argv, argv+argc, "--topic_prefix", topic_prefix);
            cout << cout_indent[0] << "topic_prefix: " << topic_prefix << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--use_original_timestamps")) {
            use_original_timestamps = true;
            cout << cout_indent[0] << "use_original_timestamps" << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--pose_enu_frame")) {
            GetCmdOption(argv, argv+argc, "--pose_enu_frame", pose_enu_frame);
            cout << cout_indent[0] << "pose_enu_frame: " << pose_enu_frame << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--remove_pos_offset")) {
            remove_pos_offset = true;
            cout << cout_indent[0] << "remove_pos_offset: enabled" << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--publish_to_tf")) {
            publish_to_tf = true;
            cout << cout_indent[0] << "publish_to_tf: enabled" << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--published_frame")) {
            GetCmdOption(argv, argv+argc, "--published_frame", published_frame);
            cout << cout_indent[0] << "published_frame: " << published_frame << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--publish_path")) {
            publish_path = true;
            cout << cout_indent[0] << "publish_path: enabled" << endl;
            any_option_set = true;
        }
        // flag odom_only is dominant over pose_only, which is dominant over
        //  enable_odom and enable_pose
        if (pad_customer_code==0) {
            if (CmdOptionExists(argv, argv+argc, "--odom_only")) {
                odom_only = true;
                enable_odom = true;
                cout << cout_indent[0] << "odom_only: set" << endl;
                any_option_set = true;
            } else {
                if (CmdOptionExists(argv, argv+argc, "--pose_only")) {
                    pose_only = true;
                    enable_pose = true;
                    cout << cout_indent[0] << "pose_only: set" << endl;
                    any_option_set = true;
                } else {
                    if (CmdOptionExists(argv, argv+argc, "--enable_odom")) {
                        enable_odom = true;
                        cout << cout_indent[0] << "enable_odom: set" << endl;
                        any_option_set = true;
                    }
                    if (CmdOptionExists(argv, argv+argc, "--enable_pose")) {
                        enable_pose = true;
                        cout << cout_indent[0] << "enable_pose: set" << endl;
                        any_option_set = true;
                    }
                }
            }
            if (enable_odom) {
                decode_filters = true;
            }
        } else {
            cout << cout_indent[0] << "specific customer_code: " << pad_customer_code << endl;
            cout << cout_indent[0] << "--> topics not available: /odom, /ang_rate, /pose" << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--publisher_queue_size")) {
            GetCmdOption(argv, argv+argc, "--publisher_queue_size", kPublisherQueueSize);
            cout << cout_indent[0] << "publisher_queue_size: " << kPublisherQueueSize << endl;
            any_option_set = true;
        }
        if (!any_option_set) {
            cout << cout_indent[0] << "no additional options set" << endl;
        }
        cout << endl;

        std::vector<std::string> rostopic_anavs_str;
        if (!odom_only && !pose_only){
            rostopic_anavs_str.push_back("/id");
            rostopic_anavs_str.push_back("/week");
            //rostopic_anavs_str.push_back("/pose_enu");
            //rostopic_anavs_str.push_back("/odom");
            rostopic_anavs_str.push_back("/tow");
            rostopic_anavs_str.push_back("/att_euler");
            rostopic_anavs_str.push_back("/att_state");
            rostopic_anavs_str.push_back("/rtk_state");
            rostopic_anavs_str.push_back("/num_sats");
            rostopic_anavs_str.push_back("/imu_calibrated");
            rostopic_anavs_str.push_back("/pos");               // before named: pos_ned
            rostopic_anavs_str.push_back("/pos_llh");
            rostopic_anavs_str.push_back("/pos_xyz");
            rostopic_anavs_str.push_back("/vel");               // before named: vel_ned
            rostopic_anavs_str.push_back("/acc");
            rostopic_anavs_str.push_back("/pos_stddev");        // before named: pos_ned_stddev
            rostopic_anavs_str.push_back("/vel_stddev");        // before named: vel_ned_stddev
            //rostopic_anavs_str.push_back("/acc_stddev");
            rostopic_anavs_str.push_back("/att_euler_stddev");
            rostopic_anavs_str.push_back("/accuracy");
    //        rostopic_anavs_str.push_back("/rtk_filter_ready4refix");
    //        rostopic_anavs_str.push_back("/gnss_reception");
        }
        if (enable_odom) {
            rostopic_anavs_str.push_back("/odom");
            if (!odom_only) {
                rostopic_anavs_str.push_back("/ang_rate");
            }
        }
        if (enable_pose) {
            rostopic_anavs_str.push_back("/pose_enu");
        }

        // add topic suffix
        for(std::vector<std::string>::size_type i=0; i < rostopic_anavs_str.size(); i++){
            rostopics_str.push_back(topic_prefix);
            rostopics_str[i] += rostopic_anavs_str[i];
        }
        if (publish_path) {
            rostopics_str.push_back("/path");
        }

        cout << "Advertise ROS topics: " << "\n";
        for (std::vector<std::string>::size_type i = 0; i < rostopics_str.size(); ++i) {
            cout << cout_indent[0] << rostopics_str[i] << "\n";
        }
        cout << endl;
    }

    else if (mode == MODE_LEAPSECONDS2ROS) {

        cout << "Options set: " << "\n";
        bool any_option_set = false;
        if (CmdOptionExists(argv, argv+argc, "--use_original_timestamps")) {
            use_original_timestamps = true;
            cout << cout_indent[0] << "use_original_timestamps" << endl;
            any_option_set = true;
        }
        if (CmdOptionExists(argv, argv+argc, "--publisher_queue_size")) {
            GetCmdOption(argv, argv+argc, "--publisher_queue_size", kPublisherQueueSize);
            cout << cout_indent[0] << "publisher_queue_size: " << kPublisherQueueSize << endl;
            any_option_set = true;
        }
        if (!any_option_set) {
            cout << cout_indent[0] << "no additional options set" << endl;
        }
        cout << endl;

        cout << "Advertise ROS topic: " << "\n";
        cout << cout_indent[0] << leapseconds_topic << "\n";
        cout << endl;
    }

    //
    // Run TCP client
    //

    /*
     * Try to reconnect for exceptions of type 'SocketException'.
     */

    // method: exponential backoff, reconnection with exponential increasing delay,
    //         until maximum delay is reached, then program terminates
    /*
    int reconnection_delay_max = 600;
    if (CmdOptionExists(argv, argv+argc, "--reconnection_delay_max")) {
        reconnection_delay = atoi(GetCmdOption(argv, argv+argc, "--reconnection_delay_max"));
    }
    bool do_reconnect = false;
    for (int reconnection_delay = 1; reconnection_delay <= reconnection_delay_max; reconnection_delay <<= 1) {
    */

    // method: reconnection with a constant reconnection_delay, infinit attempts
    int reconnection_delay = 5;    // reconnection delay in sec
    if (CmdOptionExists(argv, argv+argc, "--reconnection_delay")) {
        GetCmdOption(argv, argv+argc, "--reconnection_delay", reconnection_delay);
    }
    bool do_reconnect = false;

    while (true) {

        //cout << "reconnection_delay " << reconnection_delay << endl;

    	try {

		    // seems to be necessary to be called
		    // again in try-catch block
		    s_catch_signals ();

		    do_reconnect = false;

            cout << "\nTCP client connecting..." << endl;
            cout << cout_indent[0] << "host: " << ipaddr << endl;
            cout << cout_indent[0] << "port: " << port << endl;

		    ClientSocket client_socket ( ipaddr.c_str() , port );

		    SocketData data_recvd;

            // method: exponential backoff
            //reconnection_delay = 1;

            cout << "TCP client successfully connected.\n" << endl;
            cout << "-------------------------------------------------------------\n" << endl;
		    // Variables used for decoding messages
		    // in deserialization functions
		    sUbxDecodeState ubxState = {};      // initialize values to zero

            if (mode == MODE_DEBUG_PADFEEDER_VIO) {

		        msg_PADFeeder = new sMaplabPADFeeder;

		        while ( true ) {
		            client_socket >> data_recvd;
		            DeserializeVisUBX2ROS( data_recvd, ubxState, *msg_PADFeeder,
		                                    &VIOUBXData_callback);
		        }

            } else if (mode == MODE_DEBUG_PADGUI_IMAGE) {

		        msg_PADGUI = new sMaplabPADGUI;
		        msg_PADGUI->image.data = NULL;

		        while ( true ) {
		            client_socket >> data_recvd;
		            DeserializeVisUBX2ROS( data_recvd, ubxState, *msg_PADGUI,
		                                    &ImageUBXData_callback);
		        }

            } else if (mode == MODE_PADSENSORDATA2ROS) {

		        msg_Maplab = new sPADFeederMaplab;

		        ros::init(argc, argv, "anavs_sensordata2ros", ros::init_options::AnonymousName);
		        ros::NodeHandle n;

		        int aux_count = 0;

		        pub_imu = n.advertise<sensor_msgs::Imu>(rostopics_str[aux_count++], kPublisherQueueSize);
		        if (pub_imu_calibration) {
                    pub_imu_calibr = n.advertise<sensor_msgs::Imu>(rostopics_str[aux_count++], kPublisherQueueSize, true);  // latch = true
		        }
		        //ros::Rate loop_rate(100);
		        if (pub_time_imu_tow) {
                    //pub_tow = n.advertise<geometry_msgs::Pose2D>(rostopics_str[aux_count++], kPublisherQueueSize);
                    pub_tow = n.advertise<sensor_msgs::TimeReference>(rostopics_str[aux_count++], kPublisherQueueSize);
		        }
		        //ros::Rate tow_rate(100);
		        if (pub_odom_twist) {
                    pub_wheel = n.advertise<geometry_msgs::TwistStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
		        }
		        if (pub_odom_odometry) {
                    pub_odom = n.advertise<nav_msgs::Odometry>(rostopics_str[aux_count++], kPublisherQueueSize);
		        }
		        if (pub_can_odom_twist) {
                    pub_can_odom = n.advertise<geometry_msgs::TwistStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
		        }
//		        if (clock_enabled) {
//                    pub_clock = n.advertise<rosgraph_msgs::Clock>(rostopics_str[aux_count++], kPublisherQueueSize);
//		        }

                cout << "Decoding ANavS sensor data stream, publishing ROS messages..." << endl << endl;

		        while ( ros::ok() ) {
		            client_socket >> data_recvd;
		            DeserializeUBX2ROS( data_recvd, ubxState);
		        }

            } else if (mode == MODE_PADSOLUTION2ROS) {

//		        msg_Solution = new sPADSolution;

				ros::init(argc, argv, "anavs_solution2ros", ros::init_options::AnonymousName);
				ros::NodeHandle n;

				int aux_count = 0;

                if (!odom_only && !pose_only){
                        pub_id                  = n.advertise<std_msgs::UInt8>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_week                = n.advertise<std_msgs::UInt16>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_sol_tow             = n.advertise<sensor_msgs::TimeReference>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_att_euler           = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_att_state           = n.advertise<std_msgs::UInt8>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_rtk_state           = n.advertise<std_msgs::UInt8>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_num_sat             = n.advertise<std_msgs::UInt8>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_imu_calibrated      = n.advertise<std_msgs::Bool>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_position_ned        = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_position_llh        = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_position_xyz        = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_vel_ned             = n.advertise<geometry_msgs::Vector3Stamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_acc                 = n.advertise<geometry_msgs::Vector3Stamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_pos_ned_stddev      = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_vel_ned_stddev      = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        //pub_acc_stddev          = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_att_euler_stddev    = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                        pub_accuracy            = n.advertise<geometry_msgs::PointStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
        			    //pub_ready4refix         = n.advertise<std_msgs::Bool>(rostopics_str[aux_count++], kPublisherQueueSize);
                        //pub_gnss_rcptn          = n.advertise<std_msgs::Float64>(rostopics_str[aux_count++], kPublisherQueueSize);
                }

                if (enable_odom) {
                    pub_pose_twist_enu      = n.advertise<nav_msgs::Odometry>(rostopics_str[aux_count++], kPublisherQueueSize);
                    if (!odom_only) {
                        pub_ang_rate        = n.advertise<geometry_msgs::Vector3Stamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                    }
                }
                if (enable_pose) {
                    pub_pose_enu            = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(rostopics_str[aux_count++], kPublisherQueueSize);
                }

                if (publish_path) {
                    pub_path                = n.advertise<nav_msgs::Path>(rostopics_str[aux_count++], kPublisherQueueSizePath);
                }

//#ifdef DEBUG_MODE
//				pub_clock = n.advertise<rosgraph_msgs::Clock>("clock", 0);
//#endif

                cout << "Decoding ANavS sensor fusion solution stream, publishing ROS messages..." << endl << endl;

                // initialize multi protocol buffer
                MultiProtocolParser multi_proto_parser("RAWX");

                // add ubx subparser
                shared_ptr<UbxSubParser> ubxParser;
                ubxParser = std::make_shared<UbxSubParser>();

                shared_ptr<SubParser> ubx = ubxParser;
                multi_proto_parser.addSubParser(ubx);

                // add package listener for ubx
                shared_ptr<StreamListener> file1Ubx = std::make_shared<LambdaAdapter>(decode_ubx_solution_publish_ros);
                multi_proto_parser.addPackageListener("UBX", file1Ubx);

                // initialize buffer to read from file
                const size_t kBufferSize = 1;
                std::vector<char> buffer(kBufferSize, 0);

                while ( ros::ok() ) {
                    client_socket >> data_recvd;
                    // DeserializeUBX2ROS( data_recvd, ubxState);   // replaced by using MultiProtocolParser
                    multi_proto_parser.process((const uint8_t *)data_recvd.getPtrData(), data_recvd.getNumDataChar());
                }

            } else if (mode == MODE_LEAPSECONDS2ROS) {

                ros::init(argc, argv, "anavs_leapseconds2ros", ros::init_options::AnonymousName);
				ros::NodeHandle n;

				pub_leapseconds = n.advertise<std_msgs::Int8>(leapseconds_topic, kPublisherQueueSize);

                // initialize multi protocol buffer
                MultiProtocolParser multi_proto_parser("RAWX");

                // add ubx subparser
                shared_ptr<UbxSubParser> ubxParser;
                ubxParser = std::make_shared<UbxSubParser>();

                shared_ptr<SubParser> ubx = ubxParser;
                multi_proto_parser.addSubParser(ubx);

                // add package listener for ubx
                shared_ptr<StreamListener> file1Ubx = std::make_shared<LambdaAdapter>(decode_ubx_leapseconds_publish_ros);
                multi_proto_parser.addPackageListener("UBX", file1Ubx);

                // initialize buffer to read from file
                const size_t kBufferSize = 1;
                std::vector<char> buffer(kBufferSize, 0);

                cout << "Decoding GPS Receiver stream, publishing leapseconds ROS message..." << endl;

                while ( ros::ok() ) {
                    client_socket >> data_recvd;
                    multi_proto_parser.process((const uint8_t *)data_recvd.getPtrData(), data_recvd.getNumDataChar());
                }

            }

		} catch ( SocketException& e ) {

		    cout << e.what() << endl;

            // client reconnect for all 'SocketException' exceptions
            do_reconnect = true;

            CleanUp();

		} catch (...) {

            CleanUp();

        }
        if (s_interrupted) {
            break;
        }

        if (do_reconnect) {
            // exponential backoff
            //if (reconnection_delay <= reconnection_delay_max/2) {
                cout << "Client reconnect in " << reconnection_delay << " sec..." << endl;
                sleep(reconnection_delay);
            //O}
        } else {
            break;
        }

	}

    CleanUp();

    return 0;

}

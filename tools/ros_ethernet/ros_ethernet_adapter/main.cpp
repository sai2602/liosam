/*
 *   ANAVS ROS-Ethernet Server
 *
 *       Adapter for transfer of ROS messages via standard TCP/IP connections.
 *
 *       Dependencies:
 *           - ROS (for subscribing ROS messages)
 *
 */

/*
 * File:     main.cpp
 * Author:   Robert Bensch, Paul Faerber, ANavS GmbH
 *
 * History:  2018/04/18 - Initial file creation
 *           2018/05/09 - Added send check-connection messages when
 *                        no ROS-messages are received, to check if
 *                        client connection is still valid
 *           2018/06/20 - Changed message type naming.
 *           2019/01/11 - For message type 'ROS_PADGUI' image messages
 *                        are now send using compressed images, instead of
 *                        raw images.
 *           2019/01/16 - Initialize ros node with option AnonymousName, to
 *                        avoid identical names when node is started multiple
 *                        times.
 *           2019/02/25 - Added lidar stream for feeder/gui with the same structure
 *			             as the VIO input stream.
 *           2019/04/09 - Updated help text, added PrintHelp() function. Added simple
 *                        command line parsing. Added version number via define
 *                        'VER_ROSETHSRV'. Renamed all modes (previous msgtypes).
 *                        Updated cmdline parameters with default values for all modes.
 *           2019/04/10 - Moved 'DEBUG_MODE' define to build target 'debug-exe' in
 *                        project build options.
 *           2019/05/14 - Added Odo stream for feeder; Two different types /joint_states
 *                        from Vicky software and /wheel_odometry from AGT software
 *                        Odo stream stamped with /time_towGNSS
 *           2019/06/03 - Renamed all functions and callback functions. Implemented
 *                        functionality to switch between use of external time from time
 *                        topic or, if '--time' flag is omitted, use of ros header stamps
 *                        from odometry topics. Implemented external time access via
 *                        GetExternalTime... functions, that add ros time duration since
 *                        last external time update. Internal flag 'use_external_time' is
 *                        if '--time' topic is specified. Function 'GetTime' returns
 *                        returns ros time, or external time depending on internal flag.
 *           2019/08/13 - Added mode rosanavslidar2padfeeder. Subscribes to ros pose topic
 *                        and provides VIO data for sensor fusion via TCP server.
 *                        Updated ReadConfigFile().
 *           2019/10/28 - Read config parameters 'PAD-gear_ratio', 'PAD-wheel_radius' directly
 *                        from ANAVS.conf file. Added function ReadAnavsConfigFile().
 *           2019/12/12 - Modified spin() calls to spinOnce() in main while loops,
 *                        which should be the correct implementation. To be tested further.
 *
 */

#define VER_ROSETHSRV "0.9.12"    /* ros ethernet adapter, tcp-server mode */

#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>

//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//TCP sockets
#include "sockets/definition.h"
#include "sockets/ServerSocket.h"
#include "sockets/SocketException.h"

//ROS and Maplab-PAD type definitions
#include "ros_types.h"
#include "maplab_PAD_types.h"
#include "ubx_types.h"
#include "ublox_partial.c"

#include "common.h"

#define MODE_ROSROVIOVIO2PADFEEDER 0
#define MODE_ROSROVIOVIO2PADGUI 1
#define MODE_ROSCARTOLIDAR2PADFEEDER 2
#define MODE_ROSODO2PADFEEDER 3
#define MODE_ROSJOINTSTATESODO2PADFEEDER 4
#define MODE_ROSANAVSLIDAR2PADFEEDER 5
#define MODE_ROSPOSE2D2PADFEEDER 6
#define MODE_ROSPOSE3D2PADFEEDER 7
#define MODE_UNDEFINED -1

using namespace std;
using namespace message_filters;

// Tcp server instances
ServerSocket *serverSockInst;

string ubx_filename;

// Stores time last ROS message was received via ROS callback functions
ros::Time time_lastRosMsgRecv(0);

static bool use_external_time = true;
//static double external_time_sec = 1;
//static uint64_t external_time_us = 1;
static double external_time_sec = 0;
static ros::Time ros_time_last_update_external_time;    // initialized in main() function

static bool sub_imu = false;
static double angular_velocity_x = 0, angular_velocity_y = 0, angular_velocity_z = 0;
ros::Publisher pub_odom;
static bool pub_wheel_odometry = false;
static int16_t wheel_right_rate_rpm = 0;
static int16_t wheel_left_rate_rpm = 0;

const int kSubscriberQueueSize = 1;                 // real-time setting
const int kSubscriberQueueSizePostProc = 1000;      // post-processing setting
bool disabled_socket_server_accept = false;          // work-around to enable writing to file instead of socket connection

void ExternalTime_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
//    ROS_INFO("pub_tow_test [%f]", msg->x);
    //external_time_sec = double(msg->x+0.001);    // in sec; adding 1 millisecond???
    external_time_sec = double(msg->x);    // in sec;
    ros_time_last_update_external_time = ros::Time::now();
    // msg->y;  // ros time when sending this time message

#ifdef DEBUG_MODE
    std::cout << "Subscribed time:" << std::endl;
    std::cout << "\texternal time (s) = " << external_time_sec << std::endl;
    std::cout << "\tros time when this message was send (s) = " << msg->y << std::endl;
    std::cout << "\tcurrent ros time (s) = " << ros_time_last_update_external_time.toSec() << std::endl;
#endif // DEBUG_MODE
}

// get time (in sec) based on external time, interpolate using ROS time
double GetExternalTimeSec ()
{

    ros::Duration time_elapsed = ros::Time::now() - ros_time_last_update_external_time;

    return external_time_sec + time_elapsed.toSec();

}

// get time (in sec) based on external time, interpolate using ROS time
uint64_t GetExternalTimeUSec ()
{

    ros::Duration time_elapsed = ros::Time::now() - ros_time_last_update_external_time;

    return uint64_t((external_time_sec + time_elapsed.toSec()) * 1e6);

}

// get time from ROS time, or external time depending on switch 'use_external_time'
double GetTimeSec ()
{

    if (use_external_time) {

        return GetExternalTimeSec();

    } else {

        return ros::Time::now().toSec();

    }

}

// get time from ROS time, or external time depending on switch 'use_external_time'
uint64_t GetTimeUSec ()
{

    if (use_external_time) {

        return GetExternalTimeUSec();

    } else {

        return uint64_t(ros::Time::now().toSec() * 1e6);

    }

}

void IMUROSData_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    angular_velocity_x = msg->angular_velocity.x;
    angular_velocity_y = msg->angular_velocity.y;
    angular_velocity_z = msg->angular_velocity.z;
#ifdef DEBUG_MODE
    std::cout << "IMU message:" << std::endl;
    std::cout << "angular_velocity_x = " << angular_velocity_x << std::endl;
    std::cout << "angular_velocity_y = " << angular_velocity_y << std::endl;
    std::cout << "angular_velocity_z = " << angular_velocity_z << std::endl;
#endif // DEBUG_MODE
}

/*
 * Serializes vision message header and sends data using TCP socket
 *
 *  INPUT:
 *
 *      MSGID:  header message id
 *      pl:     payload length in bytes
 *      ssInst: ServerSocket instance
 *
 */
void SerializeHeader2TCP( const char MSGID, unsigned int pl,
                          const ServerSocket& ssInst ) {

    //
    // Set header
    //
    const char HEAD1 = SYNC1_VISION;
    const char HEAD2 = SYNC2_VISION;
    const char CLASSID = CLSID_MAPLAB;

    // Payload length type
    //      2 byte (unsigned short) little endian format
    //      4 byte (unsigned int) - used for longer payloads (raw images)
    int bPayloadLen = 2;
    if (MSGID == MSGID_PADGUI) {
        bPayloadLen = 4;
    }

    unsigned char bf[bPayloadLen];
    if (bPayloadLen == 2)
    {
        setU2(bf,(unsigned short)pl);
    } else if (bPayloadLen == 4) {
        setU4(bf,(unsigned int)pl);
    }


    //
    // Write data to socket
    //

    // Header
    ssInst << SocketData( &HEAD1, sizeof(HEAD1) )
           << SocketData( &HEAD2, sizeof(HEAD2) )
           << SocketData( &CLASSID, sizeof(CLASSID) )
           << SocketData( &MSGID, sizeof(MSGID) );
    ssInst << SocketData( &bf, sizeof(bf) );

#ifdef DEBUG_MODE
    cout << "\n*** SEND MESSAGE ***\n";
    cout << "HEAD1: " << hex << (int) HEAD1 << endl;
    cout << "HEAD2: " << hex << (int) HEAD2 << endl;
    cout << "CLASSID: " << hex << (int) CLASSID << endl;
    cout << "MSGID: " << hex << (int) MSGID << endl;

    cout.copyfmt(std::ios(NULL));

    if (bPayloadLen == 2) {
        cout << "payload length (2 bytes): " << +U2(bf) << endl;
    } else if (bPayloadLen == 4) {
        cout << "payload length (4 bytes): " << +U4(bf) << endl;
    }
#endif // DEBUG_MODE

}

/*
 * Serializes vision message header and returns socket data object
 *
 *  INPUT:
 *
 *      MSGID:  header message id
 *      pl:     payload length in bytes
 *
 *  RETURNS:
 *
 *      data:   SocketData object holding serialized header data
 *
 */
const SocketData SerializeHeader( const char MSGID, unsigned int pl ) {

    //
    // Set header
    //
    const char HEAD1 = SYNC1_VISION;
    const char HEAD2 = SYNC2_VISION;
    const char CLASSID = CLSID_MAPLAB;

    // Payload length type
    //      2 byte (unsigned short) little endian format
    //      4 byte (unsigned int) - used for longer payloads (raw images)
    int bPayloadLen = 2;
    if (MSGID == MSGID_PADGUI) {
        bPayloadLen = 4;
    }

    unsigned char bf[bPayloadLen];
    if (bPayloadLen == 2)
    {
        setU2(bf,(unsigned short)pl);
    } else if (bPayloadLen == 4) {
        setU4(bf,(unsigned int)pl);
    }


    //
    // Write data to socket
    //

    // Header
    unsigned int hl = sizeof(HEAD1) + sizeof(HEAD2) +
                      sizeof(CLASSID) + sizeof(MSGID) +
                      sizeof(bf);

    SocketData data;
    data.enableDataPrealloc(hl);
    data.setData( &HEAD1, sizeof(HEAD1) );
    data.setData( &HEAD2, sizeof(HEAD2) );
    data.setData( &CLASSID, sizeof(CLASSID) );
    data.setData( &MSGID, sizeof(MSGID) );
    data.setData( &bf, sizeof(bf) );

#ifdef DEBUG_MODE
    cout << "\n*** SEND MESSAGE ***\n";
    cout << "HEAD1: " << hex << (int) HEAD1 << endl;
    cout << "HEAD2: " << hex << (int) HEAD2 << endl;
    cout << "CLASSID: " << hex << (int) CLASSID << endl;
    cout << "MSGID: " << hex << (int) MSGID << endl;

    cout.copyfmt(std::ios(NULL));

    if (bPayloadLen == 2) {
        cout << "payload length (2 bytes): " << +U2(bf) << endl;
    } else if (bPayloadLen == 4) {
        cout << "payload length (4 bytes): " << +U4(bf) << endl;
    }
#endif // DEBUG_MODE

    return data;

}

/*
 * Serializes vision message header and returns socket data object
 *
 *  INPUT:
 *
 *      MSGID:  header message id
 *      pl:     payload length in bytes
 *
 *  RETURNS:
 *
 *      data:   SocketData object holding serialized header data
 *
 */
const SocketData SerializeHeader(const char HEAD1, const char HEAD2, const char CLASSID, const char MSGID, unsigned int pl )
{

    // Payload length type
    //      2 byte (unsigned short) little endian format
    //      4 byte (unsigned int) - used for longer payloads (raw images)
    int bPayloadLen = 2;
    if (MSGID == MSGID_PADGUI) {
        bPayloadLen = 4;
    }

    unsigned char bf[bPayloadLen];
    if (bPayloadLen == 2)
    {
        setU2(bf,(unsigned short)pl);
    } else if (bPayloadLen == 4) {
        setU4(bf,(unsigned int)pl);
    }


    //
    // Write data to socket
    //

    // Header
    unsigned int hl = sizeof(HEAD1) + sizeof(HEAD2) +
                      sizeof(CLASSID) + sizeof(MSGID) +
                      sizeof(bf);

    SocketData data;
    data.enableDataPrealloc(hl);
    data.setData( &HEAD1, sizeof(HEAD1) );
    data.setData( &HEAD2, sizeof(HEAD2) );
    data.setData( &CLASSID, sizeof(CLASSID) );
    data.setData( &MSGID, sizeof(MSGID) );
    data.setData( &bf, sizeof(bf) );

#ifdef DEBUG_MODE
    cout << "\n*** SEND MESSAGE ***\n";
    cout << "HEAD1: " << hex << (int) HEAD1 << endl;
    cout << "HEAD2: " << hex << (int) HEAD2 << endl;
    cout << "CLASSID: " << hex << (int) CLASSID << endl;
    cout << "MSGID: " << hex << (int) MSGID << endl;

    cout.copyfmt(std::ios(NULL));

    if (bPayloadLen == 2) {
        cout << "payload length (2 bytes): " << +U2(bf) << endl;
    } else if (bPayloadLen == 4) {
        cout << "payload length (4 bytes): " << +U4(bf) << endl;
    }
#endif // DEBUG_MODE

    return data;

}

/*
 * Serializes vision message from ROS-messages and sends data via TCP socket
 *
 *  The data fields are send using successive TCP send calls.
 *
 *  INPUT:
 *
 *      ros_msg0:  ROS message (nav_msgs::Odometry)
 *      ros_msg1:  ROS message (geometry_msgs::Vector3Stamped)
 *      ros_msg2:  ROS message (geometry_msgs::Vector3Stamped)
 *      ros_msg3:  ROS message (geometry_msgs::PoseStamped)
 *      ros_msg4:  ROS message (geometry_msgs::Vector3Stamped)
 *
 *  INPUT/OUTPUT:
 *
 *      ssInst: ServerSocket instance
 *
 */
void SerializeROS2TCP(const nav_msgs::Odometry::ConstPtr& ros_msg0,
                          const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg1,
                          const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg2,
                          const geometry_msgs::PoseStamped::ConstPtr& ros_msg3,
                          const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg4,
                          ServerSocket& ssInst )
{

    // Set socket option to send all data
    ssInst.setSendAll(true);


    //
    // Write header
    //

    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    SerializeHeader2TCP( MSGID, pl, ssInst );


    //
    // Write data to socket
    //

    // Payload

    // ros_msg0 odometry message: pose not used atm.
    //          in twist. is the velocity and angular rate with the covariances.
    ssInst << SocketData( &(ros_msg0->header.seq), sizeof(ros_msg0->header.seq) );
    ssInst << SocketData( &(ros_msg0->header.stamp.sec), sizeof(ros_msg0->header.stamp.sec) );
    ssInst << SocketData( &(ros_msg0->header.stamp.nsec), sizeof(ros_msg0->header.stamp.nsec) );
    ssInst << SocketData( &(ros_msg0->twist.twist.linear.x), sizeof(ros_msg0->twist.twist.linear.x) );
    ssInst << SocketData( &(ros_msg0->twist.twist.linear.y), sizeof(ros_msg0->twist.twist.linear.y) );
    ssInst << SocketData( &(ros_msg0->twist.twist.linear.z), sizeof(ros_msg0->twist.twist.linear.z) );
    ssInst << SocketData( &(ros_msg0->twist.twist.angular.x), sizeof(ros_msg0->twist.twist.angular.x) );
    ssInst << SocketData( &(ros_msg0->twist.twist.angular.y), sizeof(ros_msg0->twist.twist.angular.y) );
    ssInst << SocketData( &(ros_msg0->twist.twist.angular.z), sizeof(ros_msg0->twist.twist.angular.z) );
    ssInst << SocketData( &(ros_msg0->twist.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    ssInst << SocketData( &(ros_msg0->twist.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    ssInst << SocketData( &(ros_msg0->twist.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    ssInst << SocketData( &(ros_msg0->twist.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    ssInst << SocketData( &(ros_msg0->twist.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    ssInst << SocketData( &(ros_msg0->twist.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );

    // ros_msg1: bias Acc
    ssInst << SocketData( &(ros_msg1->vector.x), sizeof(ros_msg1->vector.x) );
    ssInst << SocketData( &(ros_msg1->vector.y), sizeof(ros_msg1->vector.y) );
    ssInst << SocketData( &(ros_msg1->vector.z), sizeof(ros_msg1->vector.z) );
    // ros_msg2: bias Gyro
    ssInst << SocketData( &(ros_msg2->vector.x), sizeof(ros_msg2->vector.x) );
    ssInst << SocketData( &(ros_msg2->vector.y), sizeof(ros_msg2->vector.y) );
    ssInst << SocketData( &(ros_msg2->vector.z), sizeof(ros_msg2->vector.z) );

    // ros_msg3: Position from Rovioli in Rovioli coordinate frame
    ssInst << SocketData( &(ros_msg3->pose.position.x), sizeof(ros_msg3->pose.position.x) );
    ssInst << SocketData( &(ros_msg3->pose.position.y), sizeof(ros_msg3->pose.position.y) );
    ssInst << SocketData( &(ros_msg3->pose.position.z), sizeof(ros_msg3->pose.position.z) );
    ssInst << SocketData( &(ros_msg3->pose.orientation.x), sizeof(ros_msg3->pose.orientation.x) );
    ssInst << SocketData( &(ros_msg3->pose.orientation.y), sizeof(ros_msg3->pose.orientation.y) );
    ssInst << SocketData( &(ros_msg3->pose.orientation.z), sizeof(ros_msg3->pose.orientation.z) );
    ssInst << SocketData( &(ros_msg3->pose.orientation.w), sizeof(ros_msg3->pose.orientation.w) );

    // ros msg4: message from health_checker
    char mp_quality = char(ros_msg4->vector.x);
    ssInst << SocketData( &(mp_quality), sizeof(mp->quality) );

#ifdef DEBUG_MODE
    cout << "ros_msg0->header.seq: " << +ros_msg0->header.seq << endl;
    cout << "ros_msg0->header.stamp.sec: " << +ros_msg0->header.stamp.sec << endl;
    cout << "ros_msg0->header.stamp.nsec: " << +ros_msg0->header.stamp.nsec << endl;
    cout << "ros_msg0->twist.twist.linear.x: " << +ros_msg0->twist.twist.linear.x << endl;
    cout << "ros_msg0->twist.twist.linear.y: " << +ros_msg0->twist.twist.linear.y << endl;
    cout << "ros_msg0->twist.twist.linear.z: " << +ros_msg0->twist.twist.linear.z << endl;
    cout << "ros_msg0->twist.twist.angular.x: " << +ros_msg0->twist.twist.angular.x << endl;
    cout << "ros_msg0->twist.twist.angular.y: " << +ros_msg0->twist.twist.angular.y << endl;
    cout << "ros_msg0->twist.twist.angular.z: " << +ros_msg0->twist.twist.angular.z << endl;
    cout << "ros_msg0->twist.convariance[0..3, 6..8, 12..14]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[0 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[6 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[12 + i] << " ";
    cout << endl;

    cout << "ros_msg0->twist.convariance[21..23, 27..29, 33..35]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[21 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[27 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[33 + i] << " ";
    cout << endl;

    cout << "ros_msg1->vector.x: " << +ros_msg1->vector.x << endl;
    cout << "ros_msg1->vector.y: " << +ros_msg1->vector.y << endl;
    cout << "ros_msg1->vector.z: " << +ros_msg1->vector.z << endl;

    cout << "ros_msg2->vector.x: " << +ros_msg2->vector.x << endl;
    cout << "ros_msg2->vector.y: " << +ros_msg2->vector.y << endl;
    cout << "ros_msg2->vector.z: " << +ros_msg2->vector.z << endl;

    cout << "ros_msg3->pose.position.x: " << +ros_msg3->pose.position.x << endl;
    cout << "ros_msg3->pose.position.y: " << +ros_msg3->pose.position.y << endl;
    cout << "ros_msg3->pose.position.z: " << +ros_msg3->pose.position.z << endl;
    cout << "ros_msg3->pose.orientation.x: " << +ros_msg3->pose.orientation.x << endl;
    cout << "ros_msg3->pose.orientation.y: " << +ros_msg3->pose.orientation.y << endl;
    cout << "ros_msg3->pose.orientation.z: " << +ros_msg3->pose.orientation.z << endl;
    cout << "ros_msg3->pose.orientation.w: " << +ros_msg3->pose.orientation.w << endl;

    cout << "ros_msg4->vector.x: " << +ros_msg4->vector.x << " --> char(): " << +mp_quality << endl;

#endif // DEBUG_MODE

}

/*
 * Serializes vision message from ROS-messages and sends data via TCP socket
 *
 *  First all data is stored in a SocketData object and then send using
 *  a single TCP send call.
 *
 *  INPUT:
 *
 *      ros_msg0:  ROS message (nav_msgs::Odometry)
 *      ros_msg1:  ROS message (geometry_msgs::Vector3Stamped)
 *      ros_msg2:  ROS message (geometry_msgs::Vector3Stamped)
 *      ros_msg3:  ROS message (geometry_msgs::PoseStamped)
 *      ros_msg4:  ROS message (geometry_msgs::Vector3Stamped)
 *
 *  INPUT/OUTPUT:
 *
 *      ssInst: ServerSocket instance
 *
 */
void SerializeROS2TCP_singleSend(const nav_msgs::Odometry::ConstPtr& ros_msg0,
                                  const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg1,
                                  const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg2,
                                  const geometry_msgs::PoseStamped::ConstPtr& ros_msg3,
                                  const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg4,
                                  ServerSocket& ssInst )
{

    //
    // Set header data
    //

    const char MSGID = MSGID_PADFEEDER;

    // Compute payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );


    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload
    // ros_msg0
    data->setData( &(ros_msg0->header.seq), sizeof(ros_msg0->header.seq) );
    data->setData(  &(ros_msg0->header.stamp.sec), sizeof(ros_msg0->header.stamp.sec) );
    data->setData(  &(ros_msg0->header.stamp.nsec), sizeof(ros_msg0->header.stamp.nsec) );
    data->setData(  &(ros_msg0->twist.twist.linear.x), sizeof(ros_msg0->twist.twist.linear.x) );
    data->setData(  &(ros_msg0->twist.twist.linear.y), sizeof(ros_msg0->twist.twist.linear.y) );
    data->setData(  &(ros_msg0->twist.twist.linear.z), sizeof(ros_msg0->twist.twist.linear.z) );
    data->setData(  &(ros_msg0->twist.twist.angular.x), sizeof(ros_msg0->twist.twist.angular.x) );
    data->setData(  &(ros_msg0->twist.twist.angular.y), sizeof(ros_msg0->twist.twist.angular.y) );
    data->setData(  &(ros_msg0->twist.twist.angular.z), sizeof(ros_msg0->twist.twist.angular.z) );
    data->setData(  &(ros_msg0->twist.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg0->twist.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg0->twist.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg0->twist.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg0->twist.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg0->twist.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );

    // ros_msg1
    data->setData(  &(ros_msg1->vector.x), sizeof(ros_msg1->vector.x) );
    data->setData(  &(ros_msg1->vector.y), sizeof(ros_msg1->vector.y) );
    data->setData(  &(ros_msg1->vector.z), sizeof(ros_msg1->vector.z) );
    // ros_msg2
    data->setData(  &(ros_msg2->vector.x), sizeof(ros_msg2->vector.x) );
    data->setData(  &(ros_msg2->vector.y), sizeof(ros_msg2->vector.y) );
    data->setData(  &(ros_msg2->vector.z), sizeof(ros_msg2->vector.z) );

    // ros_msg3
    data->setData(  &(ros_msg3->pose.position.x), sizeof(ros_msg3->pose.position.x) );
    data->setData(  &(ros_msg3->pose.position.y), sizeof(ros_msg3->pose.position.y) );
    data->setData(  &(ros_msg3->pose.position.z), sizeof(ros_msg3->pose.position.z) );
    data->setData(  &(ros_msg3->pose.orientation.x), sizeof(ros_msg3->pose.orientation.x) );
    data->setData(  &(ros_msg3->pose.orientation.y), sizeof(ros_msg3->pose.orientation.y) );
    data->setData(  &(ros_msg3->pose.orientation.z), sizeof(ros_msg3->pose.orientation.z) );
    data->setData(  &(ros_msg3->pose.orientation.w), sizeof(ros_msg3->pose.orientation.w) );

    // ros msg4
    char mp_quality = char(ros_msg4->vector.x);
    data->setData(  &(mp_quality), sizeof(mp->quality) );


    //
    // Send data via ServerSocket
    //

    ssInst.setSendAll(true);
    ssInst << *data;


    // Delete data
    delete (SocketData*) data;

#ifdef DEBUG_MODE
    cout << "ros_msg0->header.seq: " << +ros_msg0->header.seq << endl;
    cout << "ros_msg0->header.stamp.sec: " << +ros_msg0->header.stamp.sec << endl;
    cout << "ros_msg0->header.stamp.nsec: " << +ros_msg0->header.stamp.nsec << endl;
    cout << "ros_msg0->twist.twist.linear.x: " << +ros_msg0->twist.twist.linear.x << endl;
    cout << "ros_msg0->twist.twist.linear.y: " << +ros_msg0->twist.twist.linear.y << endl;
    cout << "ros_msg0->twist.twist.linear.z: " << +ros_msg0->twist.twist.linear.z << endl;
    cout << "ros_msg0->twist.twist.angular.x: " << +ros_msg0->twist.twist.angular.x << endl;
    cout << "ros_msg0->twist.twist.angular.y: " << +ros_msg0->twist.twist.angular.y << endl;
    cout << "ros_msg0->twist.twist.angular.z: " << +ros_msg0->twist.twist.angular.z << endl;
    cout << "ros_msg0->twist.convariance[0..3, 6..8, 12..14]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[0 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[6 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[12 + i] << " ";
    cout << endl;

    cout << "ros_msg0->twist.convariance[21..23, 27..29, 33..35]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[21 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[27 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg0->twist.covariance[33 + i] << " ";
    cout << endl;

    cout << "ros_msg1->vector.x: " << +ros_msg1->vector.x << endl;
    cout << "ros_msg1->vector.y: " << +ros_msg1->vector.y << endl;
    cout << "ros_msg1->vector.z: " << +ros_msg1->vector.z << endl;

    cout << "ros_msg2->vector.x: " << +ros_msg2->vector.x << endl;
    cout << "ros_msg2->vector.y: " << +ros_msg2->vector.y << endl;
    cout << "ros_msg2->vector.z: " << +ros_msg2->vector.z << endl;

    cout << "ros_msg3->pose.position.x: " << +ros_msg3->pose.position.x << endl;
    cout << "ros_msg3->pose.position.y: " << +ros_msg3->pose.position.y << endl;
    cout << "ros_msg3->pose.position.z: " << +ros_msg3->pose.position.z << endl;
    cout << "ros_msg3->pose.orientation.x: " << +ros_msg3->pose.orientation.x << endl;
    cout << "ros_msg3->pose.orientation.y: " << +ros_msg3->pose.orientation.y << endl;
    cout << "ros_msg3->pose.orientation.z: " << +ros_msg3->pose.orientation.z << endl;
    cout << "ros_msg3->pose.orientation.w: " << +ros_msg3->pose.orientation.w << endl;

    cout << "ros_msg4->vector.x: " << +ros_msg4->vector.x << " --> char(): " << +mp_quality << endl;

#endif // DEBUG_MODE

}

/*
 * Serializes vision message from ROS-messages and sends data via TCP socket
 *
 *  The data fields are send using successive TCP send calls.
 *
 *  INPUT:
 *
 *      ros_msg:  ROS message (sensor_msgs::Image)
 *
 *  INPUT/OUTPUT:
 *
 *      ssInst: ServerSocket instance
 *
 */
 /*
void SerializeROS2TCP( const sensor_msgs::Image::ConstPtr& ros_msg,
                           ServerSocket& ssInst )
{
    // Set socket option to send all data
    ssInst.setSendAll(true);


    //
    // Write header
    //

    const char MSGID = MSGID_PADGUI;

    // Payload length
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADGUI* mp = new sMaplabPADGUI;
    unsigned int pl = sizeof(mp->image.header.seq) + sizeof(mp->image.header.stamp.sec) + sizeof(mp->image.header.stamp.nsec) +
                      sizeof(mp->image.height) + sizeof(mp->image.width) +
                      sizeof(mp->image.is_bigendian) + sizeof(mp->image.step) +
                      ros_msg->data.size();
    delete mp;

    SerializeHeader2TCP( MSGID, pl, ssInst );


    //
    // Write data to socket
    //

    // Payload
    ssInst << SocketData( &(ros_msg->header.seq), sizeof(ros_msg->header.seq) );
    ssInst << SocketData( &(ros_msg->header.stamp.sec), sizeof(ros_msg->header.stamp.sec) );
    ssInst << SocketData( &(ros_msg->header.stamp.nsec), sizeof(ros_msg->header.stamp.nsec) );
    ssInst << SocketData( &(ros_msg->height), sizeof(ros_msg->height) );
    ssInst << SocketData( &(ros_msg->width), sizeof(ros_msg->width) );
    ssInst << SocketData( &(ros_msg->is_bigendian), sizeof(ros_msg->is_bigendian) );
    ssInst << SocketData( &(ros_msg->step), sizeof(ros_msg->step) );
    // ros_msg->data is a std::vector<>, needs special treatment
    ssInst << SocketData( ros_msg->data.data(), ros_msg->data.size() );

#ifdef DEBUG_MODE
    cout << "ros_msg->header.seq: " << +ros_msg->header.seq << endl;
    cout << "ros_msg->header.stamp.sec: " << +ros_msg->header.stamp.sec << endl;
    cout << "ros_msg->header.stamp.nsec: " << +ros_msg->header.stamp.nsec << endl;
    cout << "ros_msg->height: " << +ros_msg->height << endl;
    cout << "ros_msg->width: " << +ros_msg->width << endl;
    cout << "ros_msg->is_bigendian: " << +ros_msg->is_bigendian << endl;
    cout << "ros_msg->step: " << +ros_msg->step << endl;
    cout << "ros_msg->data[]" << " : " << +ros_msg->data.size() << endl;
    for (int i = 0; i < 20; ++i) {
        cout << +ros_msg->data[i] << " ";
    }
    cout << endl;
#endif // DEBUG_MODE

}
*/

// send compressed images to GUI
void SerializeROS2TCP( const sensor_msgs::CompressedImage::ConstPtr& ros_msg,
                           ServerSocket& ssInst )
{
    // Set socket option to send all data
    ssInst.setSendAll(true);


    //
    // Write header
    //
    const char MSGID = MSGID_PADGUI;

    // Payload length
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADGUIcompressed* mp = new sMaplabPADGUIcompressed;
    unsigned int pl = sizeof(mp->image_compressed.header.seq) + sizeof(mp->image_compressed.header.stamp.sec) +
                      sizeof(mp->image_compressed.header.stamp.nsec) +
//                      3*sizeof(mp->image_compressed.format) +
                        ros_msg->data.size();
    delete mp;

    SerializeHeader2TCP ( MSGID, pl, ssInst );

    //
    // Write data to socket
    //

    // Payload only send JPG data + header
    ssInst << SocketData( &(ros_msg->header.seq), sizeof(ros_msg->header.seq) );
    ssInst << SocketData( &(ros_msg->header.stamp.sec), sizeof(ros_msg->header.stamp.sec) );
    ssInst << SocketData( &(ros_msg->header.stamp.nsec), sizeof(ros_msg->header.stamp.nsec) );
//    ssInst << SocketData( &(ros_msg->format), sizeof(ros_msg->format) );
//     ros_msg->data is a std::vector<>, needs special treatment
    ssInst << SocketData( ros_msg->data.data(), ros_msg->data.size() );

#ifdef DEBUG_MODE
    cout << "ros_msg->header.seq: " << +ros_msg->header.seq << endl;
    cout << "ros_msg->header.stamp.sec: " << +ros_msg->header.stamp.sec << endl;
    cout << "ros_msg->header.stamp.nsec: " << +ros_msg->header.stamp.nsec << endl;
    cout << "ros_msg->format: " << ros_msg->format << endl;
    cout << "ros_msg->data[] start" << " : " << +ros_msg->data.size() << endl;
    for (int i = 0; i < 20; ++i) {
        cout << +ros_msg->data[i] << " ";
    }
    cout << endl;
    cout << "ros_msg->data[] end: " << " : " << +ros_msg->data.size() << endl;
        for (size_t i = ros_msg->data.size()-20; i < ros_msg->data.size(); ++i) {
        cout << +ros_msg->data[i] << " ";
    }
    cout << endl;
#endif // DEBUG_MODE

}

// send lidar data to Feeder - same structure as VIO
void SerializeROS2TCP_singleSend(const geometry_msgs::PoseStamped::ConstPtr& ros_msg0,
                          const geometry_msgs::TwistStamped::ConstPtr& ros_msg1,
                          ServerSocket& ssInst )
{
    //
    // Write header
    //
    double zeroHelpScalar = 0;
    double zeroHelpArray[36] = {0};
    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );

    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg0 pose message: currently fused in ANavS MSRK LiDAR filter
    // linear and angular twist including covariances is not used at the moment

    // forward odometryB header with time stamp
    data->setData( &(ros_msg1->header.seq), sizeof(mp->odometryB.header.seq) );
    data->setData( &(ros_msg1->header.stamp.sec), sizeof(mp->odometryB.header.stamp.sec) );
    data->setData( &(ros_msg1->header.stamp.nsec), sizeof(mp->odometryB.header.stamp.nsec) );

    // twist linear and angular
    data->setData( &(ros_msg1->twist.linear.x), sizeof(mp->odometryB.twist.twist.linear.x) );
    data->setData( &(ros_msg1->twist.linear.y), sizeof(mp->odometryB.twist.twist.linear.y) );
    data->setData( &(ros_msg1->twist.linear.z), sizeof(mp->odometryB.twist.twist.linear.z) );
    data->setData( &(ros_msg1->twist.angular.x), sizeof(mp->odometryB.twist.twist.angular.x) );
    data->setData( &(ros_msg1->twist.angular.y), sizeof(mp->odometryB.twist.twist.angular.y) );
    data->setData( &(ros_msg1->twist.angular.z), sizeof(mp->odometryB.twist.twist.angular.z) );
    // omit covariance matrix (send zeros)
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );

    // omit bias accelerometer
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.z) );
    // omit bias gyroscope
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.z) );

    // pose (from Cartographer) in ROS coordinate frame
    data->setData( &(ros_msg0->pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(ros_msg0->pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(ros_msg0->pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(ros_msg0->pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(ros_msg0->pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(ros_msg0->pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(ros_msg0->pose.orientation.w), sizeof(mp->pose.orientation.w) );

    // quality, set to '1'
    char mp_quality = char(1);
    data->setData( &(mp_quality), sizeof(mp->quality) );

    //
    // Send data via ServerSocket
    //
    ssInst.setSendAll(true);
    ssInst << *data;

    // Delete data
    delete (SocketData*) data;


#ifdef DEBUG_MODE
    cout << "ros_msg1->header.seq: " << +ros_msg1->header.seq << endl;
    cout << "ros_msg1->header.stamp.sec: " << +ros_msg1->header.stamp.sec << endl;
    cout << "ros_msg1->header.stamp.nsec: " << +ros_msg1->header.stamp.nsec << endl;
    cout << "ros_msg1->twist.linear.x: " << +ros_msg1->twist.linear.x << endl;
    cout << "ros_msg1->twist.linear.y: " << +ros_msg1->twist.linear.y << endl;
    cout << "ros_msg1->twist.linear.z: " << +ros_msg1->twist.linear.z << endl;
    cout << "ros_msg1->twist.angular.x: " << +ros_msg1->twist.angular.x << endl;
    cout << "ros_msg1->twist.angular.y: " << +ros_msg1->twist.angular.y << endl;
    cout << "ros_msg1->twist.angular.z: " << +ros_msg1->twist.angular.z << endl;

    cout << "ros_msg0->pose.position.x: " << +ros_msg0->pose.position.x << endl;
    cout << "ros_msg0->pose.position.y: " << +ros_msg0->pose.position.y << endl;
    cout << "ros_msg0->pose.position.z: " << +ros_msg0->pose.position.z << endl;
    cout << "ros_msg0->pose.orientation.x: " << +ros_msg0->pose.orientation.x << endl;
    cout << "ros_msg0->pose.orientation.y: " << +ros_msg0->pose.orientation.y << endl;
    cout << "ros_msg0->pose.orientation.z: " << +ros_msg0->pose.orientation.z << endl;
    cout << "ros_msg0->pose.orientation.w: " << +ros_msg0->pose.orientation.w << endl;
#endif // DEBUG_MODE

}

// send lidar data to Feeder - same structure as VIO
void SerializeROS2TCP_singleSend(const geometry_msgs::PoseStamped::ConstPtr& ros_msg0,
                          ServerSocket& ssInst )
{
    //
    // Write header
    //
    const double zeroHelpScalar = 0;
    const double zeroHelpArray[36] = {0};
    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );

    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg0 pose message: currently fused in ANavS MSRK LiDAR filter
    // linear and angular twist including covariances is not used at the moment

    // forward pose header with time stamp
    data->setData( &(ros_msg0->header.seq), sizeof(mp->odometryB.header.seq) );
    data->setData( &(ros_msg0->header.stamp.sec), sizeof(mp->odometryB.header.stamp.sec) );
    data->setData( &(ros_msg0->header.stamp.nsec), sizeof(mp->odometryB.header.stamp.nsec) );

    // omit twist linear and angular (send zeros)
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.z) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.z) );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData( &(zeroHelpArray), sizeof(mp->odometryB.twist.covariance.angular)/3 );

    // omit bias accelerometer
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.z) );
    // omit bias gyroscope
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.z) );

    // pose (from ANavS LiDAR-SLAM) in ROS coordinate frame
    data->setData( &(ros_msg0->pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(ros_msg0->pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(ros_msg0->pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(ros_msg0->pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(ros_msg0->pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(ros_msg0->pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(ros_msg0->pose.orientation.w), sizeof(mp->pose.orientation.w) );

    // quality, set to '1'
    char mp_quality = char(1);
    data->setData( &(mp_quality), sizeof(mp->quality) );

    //
    // Send data via ServerSocket
    //
    ssInst.setSendAll(true);
    ssInst << *data;

    // Delete data
    delete (SocketData*) data;


#ifdef DEBUG_MODE
    cout << "ros_msg0->header.seq: " << +ros_msg0->header.seq << endl;
    cout << "ros_msg0->header.stamp.sec: " << +ros_msg0->header.stamp.sec << endl;
    cout << "ros_msg0->header.stamp.nsec: " << +ros_msg0->header.stamp.nsec << endl;
    cout << "ros_msg0->pose.position.x: " << +ros_msg0->pose.position.x << endl;
    cout << "ros_msg0->pose.position.y: " << +ros_msg0->pose.position.y << endl;
    cout << "ros_msg0->pose.position.z: " << +ros_msg0->pose.position.z << endl;
    cout << "ros_msg0->pose.orientation.x: " << +ros_msg0->pose.orientation.x << endl;
    cout << "ros_msg0->pose.orientation.y: " << +ros_msg0->pose.orientation.y << endl;
    cout << "ros_msg0->pose.orientation.z: " << +ros_msg0->pose.orientation.z << endl;
    cout << "ros_msg0->pose.orientation.w: " << +ros_msg0->pose.orientation.w << endl;
#endif // DEBUG_MODE

}

void SerializeROS2TCP_singleSend(const sensor_msgs::JointState::ConstPtr& ros_msg, ServerSocket& ssInst )
{
    //
    // Write header
    //
    const char HEAD1 = SYNC1_UBX;
    const char HEAD2 = SYNC2_UBX;
    const char CLSID = CLSID_UBX;
    const char MSGID = MSGID_WHEEL;


    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg: Joint states from Viking/Stihl Tirol
    //  TODO: replace by automatically getting the sum of struct member sizes

    sUbxDataWheel* mp = new sUbxDataWheel;
    unsigned int pl = sizeof(mp->towGNSS) + sizeof(mp->left_rate) + sizeof(mp->right_rate) +
                      sizeof(mp->left_current) + sizeof(mp->right_current);

    // Building header for odometry messages into feeder
    SocketData header = SerializeHeader( HEAD1, HEAD2, CLSID, MSGID, pl );

    const size_t msg_len = header.getNumDataChar() + pl + 2;  //header.getNumDataChar()-2+pl+2

    //
    // Set socket data
    // Header + payload + 2x checksum
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc(msg_len);

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg building odometry message with same structure as before for feeder.
    // I.e. tow + left_rpm + right_rpm + left_current + right_current

    uint64_t time_usec = 0;
    if (use_external_time) {
        time_usec = GetExternalTimeUSec();
    } else {
        time_usec = uint64_t(ros::Time(ros_msg->header.stamp).toSec() * 1e6);
    }
    data->setData( &(time_usec), (sizeof(mp->towGNSS)) );

    // compute revolutions per minute for Feeder:
    int16_t l_rpm = 0, r_rpm = 0;
    l_rpm = (signed int16_t)(ros_msg->velocity[0] * (-60) * gear_ratio * 0.5 / M_PI);
    r_rpm = (signed int16_t)(ros_msg->velocity[1] * (60) * gear_ratio * 0.5 / M_PI);

    wheel_right_rate_rpm = l_rpm;
    wheel_left_rate_rpm = r_rpm;

    // data->setData( &(ros_msg->velocity[0]), sizeof(mp->left_rate) );
    // data->setData( &(ros_msg->velocity[1]), sizeof(mp->right_rate) );
    data->setData( &l_rpm, sizeof(mp->left_rate) );
    data->setData( &r_rpm, sizeof(mp->right_rate) );
    data->setData( &(ros_msg->velocity[4]), sizeof(mp->left_current) );
    data->setData( &(ros_msg->velocity[5]), sizeof(mp->right_current) );

    // compute checksum
    fletcher_checksum_t chk;
    CalculateChecksum( (unsigned char*)data->getPtrData(), msg_len, chk);

    data->setData( &(chk.chkA), sizeof(chk.chkA) );
    data->setData( &(chk.chkB), sizeof(chk.chkB) );

    // Send data via ServerSocket
    ssInst.setSendAll(true);
    ssInst << *data;

    // Delete data
    delete (SocketData*) data;

#ifdef DEBUG_MODE
    if (use_external_time) {
        cout << "external time (us): ";
    } else {
        cout << "time (us): ";
    }
    cout << setprecision(16) << +time_usec << endl;
    cout << "l_rpm: " << +l_rpm << endl;
    cout << "r_rpm: " << +r_rpm << endl;
    cout << "ros_msg->velocity[4]: " << +ros_msg->velocity[4] << endl;
    cout << "ros_msg->velocity[5]: " << +ros_msg->velocity[5] << endl;
    cout << "chk.chkA: " << +chk.chkA << endl;
    cout << "chk.chkB: " << +chk.chkB << endl;
#endif // DEBUG_MODE

}

void SerializeROS2TCP_singleSend(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg0,
                          const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg1,
                          ServerSocket& ssInst )
{
    //
    // Write header
    //
    double zeroHelpScalar = 0;
//    double zeroHelpArray[36] = {0};
    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );

    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg0 pose message: currently fused in ANavS MSRK LiDAR filter
    // linear and angular twist including covariances is not used at the moment

    // forward odometryB header with time stamp
    data->setData( &(ros_msg1->header.seq), sizeof(mp->odometryB.header.seq) );
    data->setData( &(ros_msg1->header.stamp.sec), sizeof(mp->odometryB.header.stamp.sec) );
    data->setData( &(ros_msg1->header.stamp.nsec), sizeof(mp->odometryB.header.stamp.nsec) );

    // twist linear and angular
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.z) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.z) );
    // covariance matrix (diagonal of 6x6 cov matrix)
    geometry_msgs::PoseWithCovariance pose_with_cov(ros_msg1->pose);
    switchNED2ENU(pose_with_cov.covariance);          // pos variances ENU --> NED
    data->setData(  &(pose_with_cov.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(pose_with_cov.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(pose_with_cov.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    /*
    data->setData(  &(ros_msg1->pose.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    */

    // omit bias accelerometer
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.z) );
    // omit bias gyroscope
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.z) );

    // pose in ROS coordinate frame (ENU)
    switchNED2ENU(pose_with_cov.pose);          // pose ENU --> NED
    setNaN(pose_with_cov.pose.position.z);      // set NaN, to signal sensor fusion not to apply 3rd component,
                                                //  Down in NED, ie. to perform a 2d pose update only
    data->setData( &(pose_with_cov.pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(pose_with_cov.pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(pose_with_cov.pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(pose_with_cov.pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(pose_with_cov.pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(pose_with_cov.pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(pose_with_cov.pose.orientation.w), sizeof(mp->pose.orientation.w) );
    /*
    data->setData( &(ros_msg1->pose.pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(ros_msg1->pose.pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(ros_msg1->pose.pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(ros_msg1->pose.pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(ros_msg1->pose.pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.w), sizeof(mp->pose.orientation.w) );
    */

    // quality, set to '1'
    char mp_quality = char(ros_msg0->vector.x);
    data->setData( &(mp_quality), sizeof(mp->quality) );

    //
    // Send data via ServerSocket
    //
    ssInst.setSendAll(true);
    ssInst << *data;

    // Delete data
    delete (SocketData*) data;


#ifdef DEBUG_MODE
    cout << "ros_msg0->header.seq: " << +ros_msg0->header.seq << endl;
    cout << "ros_msg0->header.stamp.sec: " << +ros_msg0->header.stamp.sec << endl;
    cout << "ros_msg0->header.stamp.nsec: " << +ros_msg0->header.stamp.nsec << endl;

    cout << "ros_msg1->pose.covariance[0..3, 6..8, 12..14]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[0 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[6 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[12 + i] << " ";
    cout << endl;

    cout << "ros_msg1->pose.covariance[21..23, 27..29, 33..35]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[21 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[27 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[33 + i] << " ";
    cout << endl;

    cout << "ros_msg1->pose.pose.position.x: " << +ros_msg1->pose.pose.position.x << endl;
    cout << "ros_msg1->pose.pose.position.y: " << +ros_msg1->pose.pose.position.y << endl;
    cout << "ros_msg1->pose.pose.position.z: " << +ros_msg1->pose.pose.position.z << endl;
    cout << "ros_msg1->pose.pose.orientation.x: " << +ros_msg1->pose.pose.orientation.x << endl;
    cout << "ros_msg1->pose.pose.orientation.y: " << +ros_msg1->pose.pose.orientation.y << endl;
    cout << "ros_msg1->pose.pose.orientation.z: " << +ros_msg1->pose.pose.orientation.z << endl;
    cout << "ros_msg1->pose.pose.orientation.w: " << +ros_msg1->pose.pose.orientation.w << endl;

    cout << "ros_msg0->vector.x: " << +ros_msg0->vector.x << " --> char(): " << +mp_quality << endl;

#endif // DEBUG_MODE

}

void SerializeROS2File(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg0, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg1, string filename){
//
    // Write header
    //
    double zeroHelpScalar = 0;
//    double zeroHelpArray[36] = {0};
    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );

    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg0 pose message: currently fused in ANavS MSRK LiDAR filter
    // linear and angular twist including covariances is not used at the moment

    // forward odometryB header with time stamp
    data->setData( &(ros_msg1->header.seq), sizeof(mp->odometryB.header.seq) );
    data->setData( &(ros_msg1->header.stamp.sec), sizeof(mp->odometryB.header.stamp.sec) );
    data->setData( &(ros_msg1->header.stamp.nsec), sizeof(mp->odometryB.header.stamp.nsec) );

    // twist linear and angular
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.z) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.z) );
    // covariance matrix (diagonal of 6x6 cov matrix)
    geometry_msgs::PoseWithCovariance pose_with_cov(ros_msg1->pose);
    switchNED2ENU(pose_with_cov.covariance);          // pos variances ENU --> NED
    data->setData(  &(pose_with_cov.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(pose_with_cov.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(pose_with_cov.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(pose_with_cov.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    /*
    data->setData(  &(ros_msg1->pose.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    */

    // omit bias accelerometer
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.z) );
    // omit bias gyroscope
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.z) );

    // pose in ROS coordinate frame (ENU)
    switchNED2ENU(pose_with_cov.pose);          // pose ENU --> NED
    setNaN(pose_with_cov.pose.position.z);      // set NaN, to signal sensor fusion not to apply 3rd component,
                                                //  Down in NED, ie. to perform a 2d pose update only
    data->setData( &(pose_with_cov.pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(pose_with_cov.pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(pose_with_cov.pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(pose_with_cov.pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(pose_with_cov.pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(pose_with_cov.pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(pose_with_cov.pose.orientation.w), sizeof(mp->pose.orientation.w) );
    /*
    data->setData( &(ros_msg1->pose.pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(ros_msg1->pose.pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(ros_msg1->pose.pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(ros_msg1->pose.pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(ros_msg1->pose.pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.w), sizeof(mp->pose.orientation.w) );
    */

    // quality, set to '1'
    char mp_quality = char(ros_msg0->vector.x);
    data->setData( &(mp_quality), sizeof(mp->quality) );

    // write data to file
    auto ubx_file = ofstream(filename, ios_base::app | ios::binary);
    // ubx_file << *data->getPtrData();         // Does only write 1 byte of data ptr is pointing to!
    ubx_file.write(data->getPtrData(), data->getNumDataChar());

    // Delete data
    delete (SocketData*) data;


#ifdef DEBUG_MODE
    cout << "ros_msg0->header.seq: " << +ros_msg0->header.seq << endl;
    cout << "ros_msg0->header.stamp.sec: " << +ros_msg0->header.stamp.sec << endl;
    cout << "ros_msg0->header.stamp.nsec: " << +ros_msg0->header.stamp.nsec << endl;

    cout << "ros_msg1->pose.covariance[0..3, 6..8, 12..14]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[0 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[6 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[12 + i] << " ";
    cout << endl;

    cout << "ros_msg1->pose.covariance[21..23, 27..29, 33..35]: ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[21 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[27 + i] << " ";
    for (int i = 0; i < 3; ++i)
        cout << +ros_msg1->pose.covariance[33 + i] << " ";
    cout << endl;

    cout << "ros_msg1->pose.pose.position.x: " << +ros_msg1->pose.pose.position.x << endl;
    cout << "ros_msg1->pose.pose.position.y: " << +ros_msg1->pose.pose.position.y << endl;
    cout << "ros_msg1->pose.pose.position.z: " << +ros_msg1->pose.pose.position.z << endl;
    cout << "ros_msg1->pose.pose.orientation.x: " << +ros_msg1->pose.pose.orientation.x << endl;
    cout << "ros_msg1->pose.pose.orientation.y: " << +ros_msg1->pose.pose.orientation.y << endl;
    cout << "ros_msg1->pose.pose.orientation.z: " << +ros_msg1->pose.pose.orientation.z << endl;
    cout << "ros_msg1->pose.pose.orientation.w: " << +ros_msg1->pose.pose.orientation.w << endl;

    cout << "ros_msg0->vector.x: " << +ros_msg0->vector.x << " --> char(): " << +mp_quality << endl;

#endif // DEBUG_MODE
}

////////////////////////////////////////////////////////////////////********************************************************************



//void SerializeROS2File(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg0, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg1, string filename){
void SerializeROS2File(const geometry_msgs::PoseStamped::ConstPtr& ros_msg1, string filename){

//
    // Write header
    //
    double zeroHelpScalar = 0;
//    double zeroHelpArray[36] = {0};
    const char MSGID = MSGID_PADFEEDER;

    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg0: odometryB.twist.twist.linear, odometryB.twist.twist.angular
    //      ros_msg1: biasAcc
    //      ros_msg2: biasGyro
    //      ros_msg3: T_G_I
    //      ros_msg4: healthChecker
    //  TODO: replace by automatically getting the sum of struct member sizes
    sMaplabPADFeeder* mp = new sMaplabPADFeeder;
    unsigned int pl = sizeof(mp->odometryB.header.seq) + sizeof(mp->odometryB.header.stamp.sec) + sizeof(mp->odometryB.header.stamp.nsec) +
                      sizeof(mp->odometryB.twist.twist.linear.x) + sizeof(mp->odometryB.twist.twist.linear.y) + sizeof(mp->odometryB.twist.twist.linear.z) +
                      sizeof(mp->odometryB.twist.twist.angular.x) + sizeof(mp->odometryB.twist.twist.angular.y) + sizeof(mp->odometryB.twist.twist.angular.z) +
                      sizeof(mp->odometryB.twist.covariance.linear) + sizeof(mp->odometryB.twist.covariance.angular) +
                      sizeof(mp->biasAcc.x) + sizeof(mp->biasAcc.y) + sizeof(mp->biasAcc.z) +
                      sizeof(mp->biasGyro.x) + sizeof(mp->biasGyro.y) + sizeof(mp->biasGyro.z) +
                      sizeof(mp->pose.position.x) + sizeof(mp->pose.position.y) + sizeof(mp->pose.position.z) +
                      sizeof(mp->pose.orientation.x) + sizeof(mp->pose.orientation.y) + sizeof(mp->pose.orientation.z) + sizeof(mp->pose.orientation.w) +
                      sizeof(mp->quality);
    delete mp;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );

    //
    // Set socket data
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( header.getNumDataChar() + pl );

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg0 pose message: currently fused in ANavS MSRK LiDAR filter
    // linear and angular twist including covariances is not used at the moment

    // forward odometryB header with time stamp
    data->setData( &(ros_msg1->header.seq), sizeof(mp->odometryB.header.seq) );
    data->setData( &(ros_msg1->header.stamp.sec), sizeof(mp->odometryB.header.stamp.sec) );
    data->setData( &(ros_msg1->header.stamp.nsec), sizeof(mp->odometryB.header.stamp.nsec) );

    // twist linear and angular
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.linear.z) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->odometryB.twist.twist.angular.z) );
    // covariance matrix (diagonal of 6x6 cov matrix)
    geometry_msgs::Pose pose(ros_msg1->pose);
//    switchNED2ENU(pose_with_cov.covariance);          // pos variances ENU --> NED
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(zeroHelpScalar), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    /*
    data->setData(  &(ros_msg1->pose.covariance[0]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[6]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[12]), sizeof(mp->odometryB.twist.covariance.linear)/3 );
    data->setData(  &(ros_msg1->pose.covariance[21]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[27]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    data->setData(  &(ros_msg1->pose.covariance[33]), sizeof(mp->odometryB.twist.covariance.angular)/3 );
    */

    // omit bias accelerometer
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasAcc.z) );
    // omit bias gyroscope
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.x) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.y) );
    data->setData( &(zeroHelpScalar), sizeof(mp->biasGyro.z) );

    // pose in ROS coordinate frame (ENU)
    switchNED2ENU(pose);          // pose ENU --> NED
    //setNaN(pose_with_cov.pose.position.z);      // set NaN, to signal sensor fusion not to apply 3rd component,
                                                //  Down in NED, ie. to perform a 2d pose update only
    data->setData( &(pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(pose.orientation.w), sizeof(mp->pose.orientation.w) );
    /*
    data->setData( &(ros_msg1->pose.pose.position.x), sizeof(mp->pose.position.x) );
    data->setData( &(ros_msg1->pose.pose.position.y), sizeof(mp->pose.position.y) );
    data->setData( &(ros_msg1->pose.pose.position.z), sizeof(mp->pose.position.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.x), sizeof(mp->pose.orientation.x) );
    data->setData( &(ros_msg1->pose.pose.orientation.y), sizeof(mp->pose.orientation.y) );
    data->setData( &(ros_msg1->pose.pose.orientation.z), sizeof(mp->pose.orientation.z) );
    data->setData( &(ros_msg1->pose.pose.orientation.w), sizeof(mp->pose.orientation.w) );
    */

    //quality, set to '1'
    char mp_quality = char(1);
    data->setData( &(mp_quality), sizeof(mp->quality) );

    // write data to file
    auto ubx_file = ofstream(filename, ios_base::app | ios::binary);
    // ubx_file << *data->getPtrData();         // Does only write 1 byte of data ptr is pointing to!
    ubx_file.write(data->getPtrData(), data->getNumDataChar());

    // Delete data
    delete (SocketData*) data;


#ifdef DEBUG_MODE
    cout << "ros_msg1->header.seq: " << +ros_msg1->header.seq << endl;
    cout << "ros_msg1->header.stamp.sec: " << +ros_msg1->header.stamp.sec << endl;
    cout << "ros_msg1->header.stamp.nsec: " << +ros_msg1->header.stamp.nsec << endl;

    cout << "ros_msg1->pose.position.x: " << +ros_msg1->pose.position.x << endl;
    cout << "ros_msg1->pose.position.y: " << +ros_msg1->pose.position.y << endl;
    cout << "ros_msg1->pose.position.z: " << +ros_msg1->pose.position.z << endl;
    cout << "ros_msg1->pose.orientation.x: " << +ros_msg1->pose.orientation.x << endl;
    cout << "ros_msg1->pose.orientation.y: " << +ros_msg1->pose.orientation.y << endl;
    cout << "ros_msg1->pose.orientation.z: " << +ros_msg1->pose.orientation.z << endl;
    cout << "ros_msg1->pose.orientation.w: " << +ros_msg1->pose.orientation.w << endl;

#endif // DEBUG_MODE
}


void SerializeROS2TCP_AGT_singleSend(const sensor_msgs::JointState::ConstPtr& ros_msg, ServerSocket& ssInst )
{
    //
    // Write header
    //
    const char HEAD1 = SYNC1_UBX;
    const char HEAD2 = SYNC2_UBX;
    const char CLSID = CLSID_UBX;
    const char MSGID = MSGID_WHEEL;


    // Payload length
    //  Skip headers of following ROS messages:
    //      ros_msg: Joint states from Viking/Stihl Tirol
    //  TODO: replace by automatically getting the sum of struct member sizes

    sUbxDataWheel* mp = new sUbxDataWheel;
    unsigned int pl = sizeof(mp->towGNSS) + sizeof(mp->left_rate) + sizeof(mp->right_rate) +
                      sizeof(mp->left_current) + sizeof(mp->right_current);

    // Building header for odometry messages into feeder
    SocketData header = SerializeHeader( HEAD1, HEAD2, CLSID, MSGID, pl );

    const size_t kMsgLen = header.getNumDataChar() + pl + 2;  //header.getNumDataChar()-2+pl+2

    //
    // Set socket data
    // Header + payload + 2x checksum
    //
    SocketData* data = new SocketData();
    data->enableDataPrealloc( kMsgLen);

    // Header
    data->setData( header.getPtrData(), header.getNumDataChar() );

    // Payload

    // ros_msg building odometry message with same structure as before for feeder.
    // I.e. tow + left_rpm + right_rpm + left_current + right_current

    // todo: check why tow is send as double and not uint64???
    double time_sec = 0;
    if (use_external_time) {
        time_sec = GetExternalTimeSec();
    } else {
        time_sec = ros::Time(ros_msg->header.stamp).toSec();
    }
    data->setData( &(time_sec), (sizeof(mp->towGNSS)) );

    // compute revolutions per minute for Feeder:
    // here: counts per minute * encoder_resolution
    int16_t l_rpm = 0, r_rpm = 0;
    int16_t l_cur = 0, r_cur = 0;
    l_rpm = (signed int16_t)(ros_msg->velocity[0] / encoder_resolution);
    r_rpm = (signed int16_t)(ros_msg->velocity[1] / encoder_resolution);

    // data->setData( &(ros_msg->velocity[0]), sizeof(mp->left_rate) );
    // data->setData( &(ros_msg->velocity[1]), sizeof(mp->right_rate) );
    data->setData( &l_rpm, sizeof(mp->left_rate) );
    data->setData( &r_rpm, sizeof(mp->right_rate) );
    data->setData( &l_cur, sizeof(mp->left_current) );
    data->setData( &r_cur, sizeof(mp->right_current) );

    // compute checksum
    fletcher_checksum_t chk;
    CalculateChecksum( (unsigned char*)data->getPtrData(), kMsgLen, chk);

    data->setData( &(chk.chkA), sizeof(chk.chkA) );
    data->setData( &(chk.chkB), sizeof(chk.chkB) );

    // Send data via ServerSocket
    ssInst.setSendAll(true);
    ssInst << *data;


    // Delete data
    delete (SocketData*) data;

#ifdef DEBUG_MODE
    if (use_external_time) {
        cout << "external time (s): ";
    } else {
        cout << "time (s): ";
    }
    cout << setprecision(16) << +time_sec << endl;
    cout << "l_rpm: " << +l_rpm << endl;
    cout << "r_rpm: " << +r_rpm << endl;
    cout << "ros_msg->velocity[4]: " << +ros_msg->velocity[4] << endl;
    cout << "ros_msg->velocity[5]: " << +ros_msg->velocity[5] << endl;
    cout << "chk.chkA: " << +chk.chkA << endl;
    cout << "chk.chkB: " << +chk.chkB << endl;
#endif // DEBUG_MODE

}

/*
 * Serializes a check connection message and sends data via TCP socket
 */
void CheckConnectionMsg2TCP( ServerSocket& ssInst )
{
    // Set socket option to send all data
    ssInst.setSendAll(true);


    //
    // Write header
    //

    const char MSGID = MSGID_CHKTCPCONN;

    // Zero payload length
    unsigned int pl = 0;

    SerializeHeader2TCP( MSGID, pl, ssInst );

}

/*
 * Serializes a check connection message and sends data via TCP socket
 */
void CheckConnectionMsg2TCP_singleSend( ServerSocket& ssInst )
{

    //
    // Set header data
    //

    const char MSGID = MSGID_CHKTCPCONN;

    // Zero payload length
    unsigned int pl = 0;

    // Get header data
    SocketData header = SerializeHeader( MSGID, pl );


    //
    // Send data via ServerSocket
    //

    ssInst.setSendAll(true);
    ssInst << header;

}

// ROS callback function
void VIOROSData_callback(const nav_msgs::Odometry::ConstPtr& ros_msg0,
                             const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg1,
                             const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg2,
                             const geometry_msgs::PoseStamped::ConstPtr& ros_msg3,
                             const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg4)
{

    /*
    cout << "ros::Time::now() \t" << ros::Time::now() << endl;
    cout << "msg0 stamp.sec: \t" << ros_msg0->header.stamp.sec << "\t.nsec: " << ros_msg0->header.stamp.nsec << endl;
    cout << "msg1 stamp.sec: \t" << ros_msg1->header.stamp.sec << "\t.nsec: " << ros_msg1->header.stamp.nsec << endl;
    cout << "msg2 stamp.sec: \t" << ros_msg2->header.stamp.sec << "\t.nsec: " << ros_msg2->header.stamp.nsec << endl;
    cout << "msg3 stamp.sec: \t" << ros_msg3->header.stamp.sec << "\t.nsec: " << ros_msg3->header.stamp.nsec << endl;
    cout << "msg4 stamp.sec: \t" << ros_msg4->header.stamp.sec << "\t.nsec: " << ros_msg4->header.stamp.nsec << endl;
    */

    SerializeROS2TCP_singleSend( ros_msg0, ros_msg1, ros_msg2, ros_msg3, ros_msg4, *serverSockInst);
    //SerializeROS2TCP( ros_msg0, ros_msg1, ros_msg2, ros_msg3, ros_msg4, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}

void LidarROSData_callback(const geometry_msgs::PoseStamped::ConstPtr& ros_msg0,
                             const geometry_msgs::TwistStamped::ConstPtr& ros_msg1)
{

    SerializeROS2TCP_singleSend( ros_msg0, ros_msg1, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}

void LidarROSData_callback(const geometry_msgs::PoseStamped::ConstPtr& ros_msg0)
{

    SerializeROS2TCP_singleSend( ros_msg0, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}

// ROS callback function
/*
void ros_PAD_GUI_callback(const sensor_msgs::Image::ConstPtr& ros_msg)
{

    SerializeROS2TCP( ros_msg, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}
*/

void PADGUICompressedImageROSData_callback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg)
{

    SerializeROS2TCP( ros_msg, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}

void WheelOdometryROSData_callback(const sensor_msgs::JointState::ConstPtr& ros_msg)
{

    SerializeROS2TCP_singleSend( ros_msg, *serverSockInst);

    // Publish odometry message for Cartographer
    if (pub_wheel_odometry) {

        nav_msgs::Odometry odom_msg;
        ros::Time stamp;
        if (use_external_time) {
            stamp = ros::Time(GetExternalTimeSec());
        } else {
            stamp = ros_msg->header.stamp;
        }
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "odom";
        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;
        odom_msg.twist.twist.linear.x = ((-1)*(1.f/gear_ratio)*(double)wheel_left_rate_rpm + (1.f/gear_ratio)*(double)wheel_right_rate_rpm) * radius_wheel * M_PI / 60;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = angular_velocity_x;
        odom_msg.twist.twist.angular.y = angular_velocity_y;
        odom_msg.twist.twist.angular.z = angular_velocity_z;
        pub_odom.publish(odom_msg);

#ifdef DEBUG_MODE
    std::cout << "Publish wheel odometry message:" << std::endl;
    std::cout << "odom_msg.header.stamp = " << odom_msg.header.stamp.toSec() << std::endl;
    std::cout << "odom_msg.twist.twist.linear.x = " << odom_msg.twist.twist.linear.x << std::endl;
    std::cout << "odom_msg.twist.twist.angular.x = " << odom_msg.twist.twist.angular.x << std::endl;
    std::cout << "odom_msg.twist.twist.angular.y = " << odom_msg.twist.twist.angular.y << std::endl;
    std::cout << "odom_msg.twist.twist.angular.z = " << odom_msg.twist.twist.angular.z << std::endl;
#endif // DEBUG_MODE

    }

    time_lastRosMsgRecv = ros::Time::now();

}

void WheelOdometryAGTROSData_callback(const sensor_msgs::JointState::ConstPtr& ros_msg)
{

    SerializeROS2TCP_AGT_singleSend( ros_msg, *serverSockInst);

    time_lastRosMsgRecv = ros::Time::now();

}

void Pose2dROSData_callback(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg0,
                             const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg1)
{
    SerializeROS2TCP_singleSend( ros_msg0, ros_msg1, *serverSockInst);
    time_lastRosMsgRecv = ros::Time::now();

}

void Pose2dROSData_file_callback(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg0,
                                    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg1)
{
    cout << " writing to file" << std::endl;
    SerializeROS2File(ros_msg0, ros_msg1, ubx_filename);
    time_lastRosMsgRecv = ros::Time::now();

}

void Pose3dROSData_callback(const geometry_msgs::PoseStamped::ConstPtr& ros_msg)
{
    SerializeROS2TCP_singleSend( ros_msg, *serverSockInst);
    time_lastRosMsgRecv = ros::Time::now();

}

void Pose3dROSData_file_callback(const geometry_msgs::PoseStamped::ConstPtr& ros_msg)
{
    cout << " writing to file" << std::endl;
    SerializeROS2File(ros_msg, ubx_filename);
    time_lastRosMsgRecv = ros::Time::now();

}

void PrintHelp(char **argv, int customer_code = 0)
{
    const std::string progname( GetLastItemFromPath( argv[0]));

    cout << "\nANavS ROS-Ethernet Adapter (TCP-Server Mode)\n";
    cout << endl;
    if (customer_code!=4) {
        cout << "  Mode: rosanavslidar2padfeeder\n" << endl;
        cout << "    Subscribes ROS pose topic (from ANavS LiDAR-SLAM) and provides\n";
        cout << "    data to ANavS sensor fusion (via TCP stream)." << endl;
        cout << endl;
    }
    if (customer_code!=3 && customer_code!=4) {
        cout << "  Mode: roscartolidar2padfeeder\n" << endl;
        cout << "    Subscribes pose and velocity topic from Google Cartographer and provides\n";
        cout << "    TCP/IP stream to ANavS PAD software." << endl;
        cout << endl;
        cout << "  Mode: rosjointstatesodo2padfeeder\n" << endl;
        cout << "    Subscribes odometry topic (sensor_msgs::JointState /joint_states [rad per second]) \n";
        cout << "    from STIHL Tirol software (vicky-anavs) and provides \n";
        cout << "    TCP/IP stream to ANavS PAD software. Optionally subscribes imu topic (sensor_msgs::Imu /imu0)\n";
        cout << "    to extract gyroscope measurements and publishes odometry topic for Google Cartographer.\n";
        cout << endl;
    }
    if (customer_code==0) {
        cout << "  Mode: rosodo2padfeeder\n" << endl;
        cout << "    Subscribes odometry topic from AGT software \n";
        cout << "    (sensor_msgs::JointState: /wheel_odometry [counts per second]) \n";
        cout << "    and provides TCP/IP stream to ANavS PAD software.\n";
        cout << "    NOT TESTED WITH REAL DATA YET!\n";     //TODO: Adapt according to mode rosjointstatesodo2padfeeder.
        cout << endl;
    }
    cout << "Usage:\n\n";
    if (customer_code!=4) {
        cout << "  " << progname << " rosanavslidar2padfeeder --port <port> [--pose_topic <topic-name>]\n";
    }
    if (customer_code!=3 && customer_code!=4) {
        cout << "  " << progname << " roscartolidar2padfeeder --port <port> [--pose_topic <topic-name>] [--velocity_topic <topic-name>]\n";
        cout << "  " << progname << " rosjointstatesodo2padfeeder --port <port> [--joint_states <joint-states-topic>] [--towGNSS <time-topic>] [--imu <imu-topic>] [--odom_carto <odom-topic>]\n";
    }
    if (customer_code==0) {
        cout << "  " << progname << " rosodo2padfeeder --port <port> [--wheel_odometry <wheel-odometry-topic>] [--towGNSS <time-topic>]\n";
    }
    if (customer_code==0 || customer_code==4) {
        cout << "  " << progname << " rosroviovio2padfeeder --port <port> [--odometry <odometry-topic>] [--imu_bias_acc <imu-accelerometer-bias-topic>] [--imu_bias_gyro <imu-gyroscope-bias-topic>] [--pose_topic <topic-name>] [--healthchecker <healthchecker-topic>]\n";
        cout << "  " << progname << " rosroviovio2padgui --port <port> [--image <image-topic>]\n";
    }

    cout << "  " << progname << " rospose2d2padfeeder --port <port> [--pose_2d <topic-name>] [--pose_2d_health <topic-name>] [--writeToFile [<filename>]]\n";
    cout << "  " << progname << " rospose3d2padfeeder --port <port> [--pose_3d <topic-name>] [--writeToFile [<filename>]]\n";

    cout << "\nOptions:\n";
    cout << endl;
    cout << "  -h --help\t\t\t" << "Show help.\n";
    cout << "  --port\t\t\t" << "ANavS software server port, specified in config file (ANAVS.conf) in the input stream path 'inpstr5-path'." << endl;
    cout << endl;
    if (customer_code!=4) {
        cout << "  rosanavslidar2padfeeder:" << endl;
        cout << "  --pose_topic <topic-name>\t" << "Specify name of pose topic that is subscribed (type: geometry_msgs::PoseStamped) [default: /pose]." << endl;
        cout << endl;
    }
    if (customer_code!=3 && customer_code!=4) {
        cout << "  roscartolidar2padfeeder:" << endl;
        cout << "  --pose_topic <topic-name>\t" << "Specify name of pose topic that is subscribed (type: geometry_msgs::PoseStamped) [default: /tracked_pose]" << endl;
        cout << "  --velocity_topic <topic-name>\t" << "Specify name of velocity topic that is subscribed (type: geometry_msgs::TwistStamped) [default: /velocity]" << endl;
        cout << endl;
        cout << "  rosjointstatesodo2padfeeder:" << endl;
        cout << "  --joint_states\t" << "Joint states topic (sensor_msgs::JointState) [default: /joint_states]" << endl;
        cout << "  --time\t" << "External time topic used for stamping outgoing tcp messages (geometry_msgs::Pose2D) [default: /time_towGNSS]." << endl;
        cout << "                If not specified, use ROS message time stamps." << endl;
        cout << "  --imu\t\t" << "Subscribed imu topic for extracting gyroscope measurements (sensor_msgs::Imu) [default: /imu0]." << endl;
        cout << "  --odom_carto\t" << "Published odometry topic (nav_msgs::Odometry) [default: /odo]" << endl;
        cout << endl;
    }
    if (customer_code==0) {
        cout << "  rosodo2padfeeder:" << endl;
        cout << "  --wheel_odometry\t" << "Wheel odometry topic (sensor_msgs::JointState) [default: /wheel_odometry]" << endl;
        cout << "  --time\t" << "External time topic used for stamping outgoing tcp messages (geometry_msgs::Pose2D) [default: /time_towGNSS]." << endl;
        cout << "                If not specified, use ROS message time stamps." << endl;
        cout << endl;
    }
    if (customer_code==0 || customer_code==4) {
        cout << "  rosroviovio2padfeeder:" << endl;
        cout << "  --odometry\t" << "Odometry topic (nav_msgs::Odometry) [default: /maplab_rovio/odometry_B]" << endl;
        cout << "  --imu_bias_acc\t" << "IMU accelerometer bias topic (geometry_msgs::Vector3Stamped) [default: /maplab_rovio/bias_acc]" << endl;
        cout << "  --imu_bias_gyro\t" << "IMU gyroscope bias topic (geometry_msgs::Vector3Stamped) [default: /maplab_rovio/bias_gyro]" << endl;
        cout << "  --pose_topic <topic-name>\t" << "Specify name of pose topic that is subscribed (type: geometry_msgs::PoseStamped) [default: /maplab_rovio/T_G_I]" << endl;
        cout << "  --healthchecker\t" << "Healthchecker topic (geometry_msgs::Vector3Stamped) [default: /HealthChecker]" << endl;
        cout << endl;
        cout << "  rosroviovio2padgui:" << endl;
        cout << "  --image\t" << "Image topic, compressed (sensor_msgs::CompressedImage) [default: /image_vis].\n" << endl;
        cout << endl;
    }

        cout << "  rospose2d2padfeeder:" << endl;
        cout << "  --pose_2d\t" << "Specify name of 2D pose topic (geometry_msgs::PoseWithCovarianceStamped) [default: /pose_2d]" << endl;
        cout << "  --pose_2d_health\t" << "Specify name of 2D pose health topic (geometry_msgs::Vector3Stamped) [default: /pose_2d_health]" << endl;
        cout << "  --writeToFile [<filename>]\t" << "Flag enables writing output to UBX file, instead of UBX tcp stream. Filename is optional. [default: pose2d.ubx]" << endl;
        cout << endl;

}

/*
 * Read config file
 *
 *  Example content:
 *      rateRosMsgs 50
 *      PAD-encoder_resolution 100
 *      modes_tcpNoDelay[0] 1
 *      modes_tcpNoDelay[1] 0
 *      modes_tcpNoDelay[2] 1
 *      modes_tcpNoDelay[3] 1
 *      modes_tcpNoDelay[4] 1
 *      modes_tcpNoDelay[5] 1
*/
bool ReadConfigFile(const string filename, double& rosMsgRate, double& encoder_resolution, vector<bool>& modes_tcpNoDelay)
{
    ifstream infile(filename.c_str());

    if (infile.is_open()) {

        string key;
        double value;
        int row = 0;
        while (infile >> key && infile >> value)
        {
            //cout << key << "\t" << value << endl;
            if (row==0) {
                rosMsgRate = double(value);
            } else if (row==1) {
                encoder_resolution = double(value);
            } else {
                modes_tcpNoDelay.push_back( bool(value));
            }
            row++;
        }
        infile.close();
        return true;

    } else {
        return false;
    }
}

void CleanUp()
{
    delete serverSockInst;
    serverSockInst = NULL;
}

static int s_interrupted = 0;
static void s_signal_handler (int signal_value)
{
    cout << "\n \n************************************ \n";
    cout << "Cleaning up and exiting program ... \n";
    cout << "************************************\n";

    s_interrupted = 1;

    fflush(stdout);

    ros::shutdown();

    CleanUp();

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

int main(int argc, char **argv)
{

    cout << "\n***********************************************************\n";
    cout << "* ANavS ROS-Ethernet Adapter (TCP-Server: v" << VER_ROSETHSRV << "[" << customer_code << "])     *\n";
    cout << "***********************************************************\n\n";

#ifdef DEBUG_MODE
    cout << "Debug mode: ON\n" << endl;
#endif // DEBUG_MODE

    if (argc==2 && (CmdOptionExists(argv, argv+argc, "-h") || CmdOptionExists(argv, argv+argc, "--help"))) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    if ((argc < 4)) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    //
    // Server modes
    //
    int cidx = 1;
    const string mode_str(argv[cidx++]);

    if (!strcmp(mode_str.c_str(), "rosroviovio2padfeeder") && argc>=4) {
        mode = MODE_ROSROVIOVIO2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rosroviovio2padgui") && argc>=4) {
        mode = MODE_ROSROVIOVIO2PADGUI;
    } else if (!strcmp(mode_str.c_str(), "roscartolidar2padfeeder") && argc>=4) {
        mode = MODE_ROSCARTOLIDAR2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rosanavslidar2padfeeder") && argc>=4) {
        mode = MODE_ROSANAVSLIDAR2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rosodo2padfeeder") && argc>=4) {
        mode = MODE_ROSODO2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rosjointstatesodo2padfeeder") && argc>=4) {
        mode = MODE_ROSJOINTSTATESODO2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rospose2d2padfeeder") && argc>=4) {
        mode = MODE_ROSPOSE2D2PADFEEDER;
    } else if (!strcmp(mode_str.c_str(), "rospose3d2padfeeder") && argc>=4) {
        mode = MODE_ROSPOSE3D2PADFEEDER;
    } else {
        mode = MODE_UNDEFINED;
    }

    if (mode == MODE_UNDEFINED) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    if (!(customer_code!=4) &&
            mode==MODE_ROSANAVSLIDAR2PADFEEDER) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if (!(customer_code!=3 && customer_code!=4) &&
            (mode==MODE_ROSCARTOLIDAR2PADFEEDER || mode==MODE_ROSJOINTSTATESODO2PADFEEDER)) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if (!(customer_code==0) &&
            mode==MODE_ROSODO2PADFEEDER) {
        PrintHelp(argv, customer_code);
        return 0;
    }
    if (!(customer_code==0 || customer_code==4) &&
            (mode==MODE_ROSROVIOVIO2PADFEEDER || mode==MODE_ROSROVIOVIO2PADGUI)) {
        PrintHelp(argv, customer_code);
        return 0;
    }

    int port = 0;         // Server TCP port

    if (CmdOptionExists(argv, argv+argc, "--port")) {
        GetCmdOption(argv, argv+argc, "--port", port);
    } else {
        PrintHelp(argv, customer_code);
        return 0;
    }

    cout << "Mode: " << mode_str << "\n" << endl;
    cout << "Port: " << port << "\n" << endl;

    // Read ANavS config file
    cout << "Config file parameters (" << anavs_config_filename << "):" << endl;
     // specify parameters to read
    std::vector< std::string > param_names;
    param_names.push_back("PAD-gear_ratio");
    param_names.push_back("PAD-wheel_radius");
    std::vector<double> param_values;
    param_values.push_back(gear_ratio);
    param_values.push_back(radius_wheel);
    //param_names.push_back("PAD-wheel_distance");   // not used atm
    if (!ReadAnavsConfigFile(anavs_config_filename, param_names, param_values)) {
        cout << "\tFile cannot be opened: " << anavs_config_filename << ". Apply standard parameters." << endl;
    } else {
       gear_ratio = param_values[0];
       radius_wheel = param_values[1];
    }
    cout << cout_indent[0] << "PAD-gear_ratio: " << gear_ratio << endl;
    cout << cout_indent[0] << "PAD-wheel_radius: " << radius_wheel << endl;
    cout << endl;

    // Read config file
    const string config_filename("configs/ANAVS_ros_ethernet_server.conf");
    cout << "Config file parameters (" << config_filename << "):" << endl;
    double rateRosMsgs = 50;
    vector<bool> modes_tcpNoDelay;
    if (!ReadConfigFile(config_filename, rateRosMsgs, encoder_resolution, modes_tcpNoDelay)) {
        cout << cout_indent[0] << "File cannot be opened: " << config_filename << ". Apply standard parameters." << endl;
        modes_tcpNoDelay.push_back(1);
        modes_tcpNoDelay.push_back(0);
        modes_tcpNoDelay.push_back(1);
        modes_tcpNoDelay.push_back(1);
        modes_tcpNoDelay.push_back(1);
        modes_tcpNoDelay.push_back(1);
    }
    cout << cout_indent[0] << "rateRosMsgs: " << rateRosMsgs << endl;
	cout << cout_indent[0] << "PAD-encoder_resolution: " << encoder_resolution << endl;
    for (vector<bool>::size_type i = 0; i < modes_tcpNoDelay.size(); ++i) {
        cout << cout_indent[0] << "modes_tcpNoDelay[" << i << "] = " << modes_tcpNoDelay[i] << "\n";
    }
    cout << endl;

    //
    // Subscribe ROS topics
    //
    ros::init(argc, argv, mode_str.c_str(), ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // initialize time
    ros_time_last_update_external_time = ros::Time::now();

    /* Handling Ctrl-C cleanly
    reference: http://zguide.zeromq.org/cpp:interrupt */
    s_catch_signals ();

    //
    // Subscribers for SINGLE-TOPIC subscription
    //
    ros::Subscriber sub_0_single;

    // Subscribers for Odo + towGNSS to feeder
    ros::Subscriber sub_1_single;
    ros::Subscriber sub_2_single;
    ros::Subscriber sub_3_single;

    //
    // Subscribers for MULTI-TOPIC subscription
    // performs time synchronization, based on timestamps
    //
    // ADD MORE SUBSCRIBERS IF NEEDED
    //
    typedef nav_msgs::Odometry T0;
    typedef geometry_msgs::Vector3Stamped T1;
    typedef geometry_msgs::PoseStamped T2;
    typedef geometry_msgs::TwistStamped T3;
    typedef geometry_msgs::PoseWithCovarianceStamped T4;

    message_filters::Subscriber<T0> sub_0;
    message_filters::Subscriber<T1> sub_1;
    message_filters::Subscriber<T1> sub_2;
    message_filters::Subscriber<T2> sub_3;
    message_filters::Subscriber<T1> sub_4;
    message_filters::Subscriber<T3> sub_5;
    message_filters::Subscriber<T4> sub_6;
    //TimeSynchronizer<T0, T1> sync(sub_0, sub_1, 10);
    // Output for Rovioli/Maplab to Feeder
    typedef sync_policies::ApproximateTime<T0, T1, T1, T2, T1> MySyncPolicy;

    // Output for Cartographer to Feeder
    typedef sync_policies::ApproximateTime<T2, T3> MySyncPolicyLidar;

    // Sync policy for pose_2d and pose_2d_health messages
    typedef sync_policies::ApproximateTime<T1, T4> MySyncPolicyPose2d;

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10));
    Synchronizer<MySyncPolicyLidar> syncLidar(MySyncPolicyLidar(10));
    Synchronizer<MySyncPolicyPose2d> syncPose2d(MySyncPolicyPose2d(10));


    //
    // Server configurations:
    //
    //    ROS-Ethernet streaming for:
    //
    //    1) "ROS_PADFeeder": Results from vision module --> Target: PAD feeder software for sensor fusion / PAD GUI for visualization
    //    2) "ROS_PADGUI"   : Raw camera images + features from vision module --> Target: PAD GUI for visualization
    //
    if (mode == MODE_ROSROVIOVIO2PADFEEDER) {

        // ROS_PAD-feeder
        string odom_rovio_topic("/maplab_rovio/odometry_B");
        string imu_bias_acc_topic("/maplab_rovio/bias_acc");
        string imu_bias_gyro_topic("/maplab_rovio/bias_gyro");
        string pose_topic("/maplab_rovio/T_G_I");
        string healthchecker_topic("/HealthChecker");

        if (CmdOptionExists(argv, argv+argc, "--odometry")) {
            GetCmdOption(argv, argv+argc, "--odometry", odom_rovio_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--imu_bias_acc")) {
            GetCmdOption(argv, argv+argc, "--imu_bias_acc", imu_bias_acc_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--imu_bias_gyro")) {
            GetCmdOption(argv, argv+argc, "--imu_bias_gyro", imu_bias_gyro_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--pose_topic")) {
            GetCmdOption(argv, argv+argc, "--pose_topic", pose_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--healthchecker")) {
            GetCmdOption(argv, argv+argc, "--healthchecker", healthchecker_topic);
        }

        /*
        cout << "Published ROS topics: " << "\n";
        cout << "   " << odom_rovio_topic << " (odometry topic)" << endl;
        cout << "   " << imu_bias_acc_topic << " (IMU accelerometer bias topic)" << endl;
        cout << "   " << imu_bias_gyro_topic << " (IMU gyroscope bias topic)" << endl;
        cout << "   " << pose_topic << " (pose topic)" << endl;
        cout << "   " << healthchecker_topic << " (healthchecker topic)" << endl;
        */

        if (modes_tcpNoDelay[mode]) {
            sub_0.subscribe(n, odom_rovio_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
            sub_1.subscribe(n, imu_bias_acc_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
            sub_2.subscribe(n, imu_bias_gyro_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
            sub_3.subscribe(n, pose_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
            sub_4.subscribe(n, healthchecker_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
        } else {
            sub_0.subscribe(n, odom_rovio_topic, kSubscriberQueueSize);
            sub_1.subscribe(n, imu_bias_acc_topic, kSubscriberQueueSize);
            sub_2.subscribe(n, imu_bias_gyro_topic, kSubscriberQueueSize);
            sub_3.subscribe(n, pose_topic, kSubscriberQueueSize);
            sub_4.subscribe(n, healthchecker_topic, kSubscriberQueueSize);
        }

        sync.connectInput(sub_0, sub_1, sub_2, sub_3, sub_4);
        sync.registerCallback(boost::bind(&VIOROSData_callback, _1, _2, _3, _4, _5));

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_0.getTopic() << endl;
        cout << cout_indent[0] << sub_1.getTopic() << endl;
        cout << cout_indent[0] << sub_2.getTopic() << endl;
        cout << cout_indent[0] << sub_3.getTopic() << endl;
        cout << cout_indent[0] << sub_4.getTopic() << "\n" << endl;

    } else if (mode == MODE_ROSROVIOVIO2PADGUI) {

        // ROS_PAD-GUI

        string image_topic("/image_vis");

        if (CmdOptionExists(argv, argv+argc, "--image")) {
            GetCmdOption(argv, argv+argc, "--image", image_topic);
        }

        /*
        cout << "Published ROS topics: " << "\n";
        cout << "   " << image_topic << " (image topic)" << endl;
        */

        //sub_0_single = n.subscribe(image_topic, 1, ros_PAD_GUI_callback);
        sub_0_single = n.subscribe(image_topic + "/compressed", kSubscriberQueueSize, PADGUICompressedImageROSData_callback);

       cout << "Subscribed ROS topics:" << endl;
       cout << cout_indent[0] << sub_0_single.getTopic() << "\n" << endl;

    } else if (mode == MODE_ROSCARTOLIDAR2PADFEEDER) {

        string pose_topic("/tracked_pose");
        string velocity_topic("/velocity");

        if (CmdOptionExists(argv, argv+argc, "--pose_topic")) {
            GetCmdOption(argv, argv+argc, "--pose_topic", pose_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--velocity")) {
            GetCmdOption(argv, argv+argc, "--velocity", velocity_topic);
        }

        if (modes_tcpNoDelay[mode]) {
            sub_3.subscribe(n, pose_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
            sub_5.subscribe(n, velocity_topic, kSubscriberQueueSize, ros::TransportHints().tcpNoDelay());
        } else {
            sub_3.subscribe(n, pose_topic, kSubscriberQueueSize);
            sub_5.subscribe(n, velocity_topic, kSubscriberQueueSize);
        }

        syncLidar.connectInput(sub_3, sub_5);
        syncLidar.registerCallback(boost::bind(&LidarROSData_callback, _1, _2));

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_3.getTopic() << endl;
        cout << cout_indent[0] << sub_5.getTopic() << "\n" <<  endl;

    } else if (mode == MODE_ROSANAVSLIDAR2PADFEEDER) {

        string pose_topic("/pose");

        if (CmdOptionExists(argv, argv+argc, "--pose_topic")) {
            GetCmdOption(argv, argv+argc, "--pose_topic", pose_topic);
        }

        //sub_0_single = n.subscribe(pose_topic, 1, LidarROSData_callback);

        if (modes_tcpNoDelay[mode]) {
            sub_0_single = n.subscribe(pose_topic, kSubscriberQueueSize, LidarROSData_callback, ros::TransportHints().tcpNoDelay());
        } else {
            sub_0_single = n.subscribe(pose_topic, kSubscriberQueueSize, LidarROSData_callback);
        }

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_0_single.getTopic() << "\n" << endl;

    } else if (mode == MODE_ROSJOINTSTATESODO2PADFEEDER) {

        string joint_states_topic("/joint_states");
        string time_topic("/time_towGNSS");
        string imu_topic("/imu0");
        string odom_carto_topic("/odo");

        if (CmdOptionExists(argv, argv+argc, "--joint_states")) {
            GetCmdOption(argv, argv+argc, "--joint_states", joint_states_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--time")) {
            // use_external_time = true;
            GetCmdOption(argv, argv+argc, "--time", time_topic);
        } else {
            // use ROS time
            use_external_time = false;
        }
        if (CmdOptionExists(argv, argv+argc, "--imu")) {
            sub_imu = true;
            GetCmdOption(argv, argv+argc, "--imu", imu_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--odom_carto")) {
            pub_wheel_odometry = true;
            GetCmdOption(argv, argv+argc, "--odom_carto", odom_carto_topic);
        }

        if (modes_tcpNoDelay[mode]) {
            sub_1_single = n.subscribe(time_topic, kSubscriberQueueSize, ExternalTime_callback, ros::TransportHints().tcpNoDelay());
            sub_0_single = n.subscribe(joint_states_topic, kSubscriberQueueSize, WheelOdometryROSData_callback, ros::TransportHints().tcpNoDelay());
            if (sub_imu) {
                sub_2_single = n.subscribe(imu_topic, kSubscriberQueueSize, IMUROSData_callback, ros::TransportHints().tcpNoDelay());
            }
        } else {
            sub_1_single = n.subscribe(time_topic, kSubscriberQueueSize, ExternalTime_callback);
            sub_0_single = n.subscribe(joint_states_topic, kSubscriberQueueSize, WheelOdometryROSData_callback);
            if (sub_imu) {
                sub_2_single = n.subscribe(imu_topic, kSubscriberQueueSize, IMUROSData_callback);
            }
        }

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_0_single.getTopic() << endl;
        cout << cout_indent[0] << sub_1_single.getTopic() << endl;
        if (sub_imu) {
            cout << cout_indent[0] << sub_2_single.getTopic() << endl;
        }
        if (pub_wheel_odometry) {
            pub_odom = n.advertise<nav_msgs::Odometry>(odom_carto_topic, 0);
            cout << "Advertised ROS topics: " << "\n";
            cout << cout_indent[0] << odom_carto_topic << " (odometry topic (Cartographer))" << endl;
        }
        cout << "\n" << endl;

    } else if (mode == MODE_ROSODO2PADFEEDER) {

        string wheel_odometry_topic("/wheel_odometry");
        string time_topic("/time_towGNSS");

        if (CmdOptionExists(argv, argv+argc, "--wheel_odometry")) {
            GetCmdOption(argv, argv+argc, "--wheel_odometry", wheel_odometry_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--time")) {
            // use_external_time = true;
            GetCmdOption(argv, argv+argc, "--time", time_topic);
        } else {
            // use ROS time
            use_external_time = false;
        }

        if (modes_tcpNoDelay[mode]) {
            sub_1_single = n.subscribe(time_topic, kSubscriberQueueSize, ExternalTime_callback, ros::TransportHints().tcpNoDelay());
            sub_0_single = n.subscribe(wheel_odometry_topic, kSubscriberQueueSize, WheelOdometryAGTROSData_callback, ros::TransportHints().tcpNoDelay());
        } else {
            sub_1_single = n.subscribe(time_topic, kSubscriberQueueSize, ExternalTime_callback);
            sub_0_single = n.subscribe(wheel_odometry_topic, kSubscriberQueueSize, WheelOdometryAGTROSData_callback);
        }

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_0_single.getTopic() << endl;
        cout << cout_indent[0] << sub_1_single.getTopic() << "\n" << endl;

    } else if (mode == MODE_ROSPOSE2D2PADFEEDER) {

        string pose_2d_topic("/pose_2d");
        string pose_2d_health_topic("/pose_2d_health");
        string write_to_filename("pose2d.ubx");

        if (CmdOptionExists(argv, argv+argc, "--pose_2d")) {
            GetCmdOption(argv, argv+argc, "--pose_2d", pose_2d_topic);
        }
        if (CmdOptionExists(argv, argv+argc, "--pose_2d_health")) {
            GetCmdOption(argv, argv+argc, "--pose_2d_health", pose_2d_health_topic);
        }
        int sub_queue_size = kSubscriberQueueSize;
        if (CmdOptionExists(argv, argv+argc, "--writeToFile")) {
            GetCmdOption(argv,argv+argc, "--writeToFile", write_to_filename);
            sub_queue_size = kSubscriberQueueSizePostProc;
            disabled_socket_server_accept = true;
        }

        if (modes_tcpNoDelay[mode]) {
            sub_1.subscribe(n, pose_2d_health_topic, sub_queue_size, ros::TransportHints().tcpNoDelay());
            sub_6.subscribe(n, pose_2d_topic, sub_queue_size, ros::TransportHints().tcpNoDelay());
        } else {
            sub_1.subscribe(n, pose_2d_health_topic, sub_queue_size);
            sub_6.subscribe(n, pose_2d_topic, sub_queue_size);
        }


        syncPose2d.connectInput(sub_1, sub_6);

        if (CmdOptionExists(argv, argv+argc, "--writeToFile")) {
            ubx_filename = write_to_filename;
            syncPose2d.registerCallback(boost::bind(&Pose2dROSData_file_callback, _1, _2));
        } else{
            syncPose2d.registerCallback(boost::bind(&Pose2dROSData_callback, _1, _2));
        }

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_1.getTopic() << endl;
        cout << cout_indent[0] << sub_6.getTopic() << "\n" <<  endl;

    } else if (mode == MODE_ROSPOSE3D2PADFEEDER) {

        string pose_3d_topic("/pose_3d");
        string write_to_filename("pose3d.ubx");

        if (CmdOptionExists(argv, argv+argc, "--pose_3d")) {
            GetCmdOption(argv, argv+argc, "--pose_3d", pose_3d_topic);
        }
        int sub_queue_size = kSubscriberQueueSize;
        if (CmdOptionExists(argv, argv+argc, "--writeToFile")) {
            GetCmdOption(argv,argv+argc, "--writeToFile", write_to_filename);
            sub_queue_size = kSubscriberQueueSizePostProc;
            disabled_socket_server_accept = true;
        }

        if (modes_tcpNoDelay[mode] && CmdOptionExists(argv, argv+argc, "--writeToFile")) {
            ubx_filename = write_to_filename;
            sub_3_single = n.subscribe(pose_3d_topic, kSubscriberQueueSize, Pose3dROSData_file_callback, ros::TransportHints().tcpNoDelay());
        }
        else if (CmdOptionExists(argv, argv+argc, "--writeToFile")) {
            ubx_filename = write_to_filename;
            sub_3_single = n.subscribe(pose_3d_topic, kSubscriberQueueSize, Pose3dROSData_file_callback);
        }
        else if (modes_tcpNoDelay[mode]) {
            sub_3_single = n.subscribe(pose_3d_topic, kSubscriberQueueSize, Pose3dROSData_callback, ros::TransportHints().tcpNoDelay());
        }
        else {
            sub_3_single = n.subscribe(pose_3d_topic, kSubscriberQueueSize, Pose3dROSData_callback);
        }

        cout << "Subscribed ROS topics:" << endl;
        cout << cout_indent[0] << sub_3_single.getTopic() << "\n" <<  endl;
    }

    else {

        cout << "Mode unknown." << endl;

        ros::shutdown();

        return 0;

    }


    //
    // Run TCP server
    //
    const double rateCheckConnMsgs = 0.33;  // [Hz]
    const double waitForRosMsgs_sec = 3;

    try {

        // Create TCP server socket
        ServerSocket server ( port);

        cout << "TCP server running (port: " << port << ")" << endl;

        while ( ros::ok() ) {

            try {

                // Create new socket for accepting incoming connections
                serverSockInst = new ServerSocket;

                cout << "TCP server accepting client connections..." << endl << endl;

                if (!disabled_socket_server_accept) {
                    server.accept ( *serverSockInst );
                }

                //cout << "TCP connection established" << endl;

                // run ros "loop", "activates" callback functions
                //ros::spin();

                ros::Rate r(rateRosMsgs);
                ros::Duration timeSince_lastRosMsgRecv(0);

                while ( ros::ok() )
                {
                    ros::spinOnce();

                    timeSince_lastRosMsgRecv = ros::Time::now() - time_lastRosMsgRecv;
                    //cout << "timeSince_lastRosMsgRecv: " << timeSince_lastRosMsgRecv.toSec() << endl;

                    if ( timeSince_lastRosMsgRecv.toSec() > waitForRosMsgs_sec ) {

                        // Start sending check TCP connection messages
                        //cout << "No ROS messages received within the last 3 sec." << endl;
                        //cout << "Sending check TCP connection messages..." << endl;

                        ros::Rate r2(rateCheckConnMsgs);

                        while ( ros::ok() && (timeSince_lastRosMsgRecv.toSec() > waitForRosMsgs_sec) )
                        {

                            CheckConnectionMsg2TCP_singleSend( *serverSockInst);
                            //CheckConnectionMsg2TCP( *serverSockInst);

                            ros::spinOnce();

                            timeSince_lastRosMsgRecv = ros::Time::now() - time_lastRosMsgRecv;
                            //cout << "CheckConnectionMsg2TCP_singleSend()\t timeSince_lastRosMsgRecv: " << timeSince_lastRosMsgRecv.toSec() << endl;
                            cout << "Time since last ROS message has been received: " << timeSince_lastRosMsgRecv.toSec() << endl;

                            r2.sleep();

                        }

                        cout << "Receiving ROS messages again." << endl;
                        cout << "Stop sending check TCP connection messages." << endl;

                    }

                    r.sleep();

                }


            } catch ( SocketException& e ) {

                cout << e.what() << endl;

            }

            CleanUp();

        }


    } catch (SocketException& e) {

            cout << e.what() << endl;

    }


    ros::shutdown();

    CleanUp();

    return 0;

}


/*------------------------------------------------------------------------------
*
*   ANAVS vision messages protocol
*
*       Protocol definitions for data transfer between
*       maplab ROS-messages and the
*       ANAVS PAD software (e.g.: PAD feeder, PAD GUI)
*
* authors : Robert Bensch, ANavS GmbH
* history : 2018/04/18 - Initial file creation
            2018/07/13 - Added wheel data
*-----------------------------------------------------------------------------*/

#ifndef MAPLAB_PAD_TYPES_H
#define MAPLAB_PAD_TYPES_H

#include "ubx_types.h"

//
// Vision message protocol header
//
#define SYNC1_VISION      0x56           // vision message sync code 1 --> ASCII: 'V'
#define SYNC2_VISION      0x49           // vision message sync code 2 --> ASCII: 'I'

#define CLSID_MAPLAB            0x03     // vision message class id: maplab

#define MSGID_PADFEEDER         0x00     // vision message id: PAD feeder
#define MSGID_PADGUI            0x01     // vision message id: PAD GUI
#define MSGID_CHKTCPCONN        0x7F     // vision message id: Check TCP connection

//
// Vision message structs
//
typedef struct
{
    sRosNavMsgsOdometryNoPose odometryB;    // linear and angular velocities, in IMU coordinate frame (B)
    sRosGeometryMsgsVector3 biasAcc;        // IMU acceleration biases
    sRosGeometryMsgsVector3 biasGyro;       // IMU gyro biases
    sRosGeometryMsgsPose pose;              // position and orientation
    char quality;                           // quality of measurements (0: unhealthy, 1: healthy)

} sMaplabPADFeeder;
//bPayloadLen = 2;

typedef struct
{
    sRosSensorMsgsImage image;

} sMaplabPADGUI;
//bPayloadLen = 4;

typedef struct
{
    sRosSensorMsgsImagecompressed image_compressed;
} sMaplabPADGUIcompressed;

typedef struct
{
    sUbxDataImu imu;
    sUbxDataWheel wheel;
    sUbxDataCanOdom canOdom;

} sPADFeederMaplab;

typedef struct
{
    sUbxDataSolution sol_ecef;

} sPADSolution;

//
// ***LEGACY*** Vision message structs
//  kept for backward compatibility
//
typedef struct
{
    sRosNavMsgsOdometryNoPose odometryB;    // linear and angular velocities, in IMU coordinate frame (B)
    sRosGeometryMsgsVector3 biasAcc;        // IMU acceleration biases
    sRosGeometryMsgsVector3 biasGyro;       // IMU gyro biases
    sRosGeometryMsgsPose pose;              // position and orientation
    // double quality;

} sMaplabPADFeeder_legacy;
//bPayloadLen = 2;

#endif // MAPLAB_PAD_TYPES_H

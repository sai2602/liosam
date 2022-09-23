/*------------------------------------------------------------------------------
*
* ROS corresponding type definitions
*
*   based on the Robot Operating System (ROS) type definitions, see:
*       http://wiki.ros.org/APIs, or for instance
*       http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
*
* authors : Robert Bensch, ANavS GmbH
* history : 2018/04/18 - Initial file creation
*-----------------------------------------------------------------------------*/

#ifndef ROS_TYPES_H
#define ROS_TYPES_H

//
// Basic types
//
typedef struct
{
        uint32_t sec;
        uint32_t nsec;
} sRosTimeBase;

typedef struct
{
       double x;
       double y;
       double z;
} sRosGeometryMsgsVector3;           //geometry_msgs::Vector3

typedef struct
{
       double x;
       double y;
       double z;
} sRosGeometryMsgsPoint;

typedef struct
{
       double x;
       double y;
       double z;
       double w;
} sRosGeometryMsgsQuaternion;

//
// Composed types
//
typedef struct
{
        uint32_t seq;
        sRosTimeBase stamp;
        //std::string frame_id;
} sRosStdMsgsHeader;                        //std_msgs::Header

typedef struct
{
        sRosStdMsgsHeader header;
        uint32_t height;
        uint32_t width;
        //std::string encoding;
        uint8_t is_bigendian;
        uint32_t step;
        uint8_t *data;
} sRosSensorMsgsImage;                      //sensor_msgs::Image

typedef struct
{
    sRosStdMsgsHeader header;
    char format;
    uint8_t *data;
} sRosSensorMsgsImagecompressed;            //sensor_msgs::CompressedImage

typedef struct
{
        sRosStdMsgsHeader header;
        sRosGeometryMsgsVector3 vector;
} sRosGeometryMsgsVector3Stamped;           //geometry_msgs::Vector3Stamped

typedef struct
{
        sRosStdMsgsHeader header;
        sRosGeometryMsgsPoint point;

} sRosGeometryMsgsPointStamped;

typedef struct
{
        sRosGeometryMsgsPoint position;
        sRosGeometryMsgsQuaternion orientation;
} sRosGeometryMsgsPose;

typedef struct
{
        sRosGeometryMsgsVector3 linear;
        sRosGeometryMsgsVector3 angular;
} sRosGeometrMsgsTwist;

typedef struct
{
        sRosGeometryMsgsPose pose;
        double covariance[36];
} sRosGeometrMsgsPoseWithCovariance;

typedef struct
{
        sRosGeometrMsgsTwist twist;
        double covariance[36];
} sRosGeometryMsgsTwistWithCovariance;

typedef struct
{
        sRosStdMsgsHeader header;
        sRosGeometrMsgsPoseWithCovariance pose;
        sRosGeometryMsgsTwistWithCovariance twist;
} sRosNavMsgsOdometry;


//
// Modified ROS types
//
typedef struct
{
        sRosGeometrMsgsTwist twist;
        struct {
            double linear[9];
            double angular[9];
        } covariance;
} sRosGeometryMsgsTwistWithCovariance9;

typedef struct
{
        sRosStdMsgsHeader header;
        sRosGeometryMsgsTwistWithCovariance9 twist;
} sRosNavMsgsOdometryNoPose;

typedef struct
{
        sRosStdMsgsHeader header;
        sRosGeometryMsgsVector3 angular_velocity;
        sRosGeometryMsgsVector3 linear_acceleration;
} sRosSensorMsgsImuNoOrientationNoCovariances;


#endif // ROS_TYPES_H

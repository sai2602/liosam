/*
 * Common functions used in the client and server application,
 *  with ROS dependencies.
 */

/*
 * File:    common_ros.h
 * Author:  Robert Bensch, ANavS GmbH
 *
 * History: 2021/04/25 - Initial file creation
 *
 * Created on April 25, 2021
 */

#ifndef COMMON_ROS_H
#define COMMON_ROS_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
 * Switches between NED and ENU frame
 *  position:    x,y,z   --> y,x,-z
 *  orientation: x,y,z,w --> y,x,-z,w
 */
void switchNED2ENU(geometry_msgs::Pose& pose)
{
    double tmp_pos_x = pose.position.x;
    pose.position.x = pose.position.y;
    pose.position.y = tmp_pos_x;
    pose.position.z = -pose.position.z;

    double tmp_orientation_x = pose.orientation.x;
    pose.orientation.x = pose.orientation.y;
    pose.orientation.y = tmp_orientation_x;
    pose.orientation.z = -pose.orientation.z;
    // pose.orientation.w                           // unchanged
}

/*
 * Modifies 6x6 covariance matrix, stored in double[36]
 *
 * - switches position cov between ENU <--> NED,
 * - switches orientation cov between (roll, pitch, heading) <--> (heading, pitch, roll)
 * - only alters diagonal elements
 */
void switchNED2ENU(boost::array<double, 36>& cov)
{
    // exchange first two elements of diagonal of upper-left 3x3 sub-matrix
    //  position cov, ENU --> NED
    double tmp_cov_0 = cov[0];
    cov[0] = cov[7];
    cov[7] = tmp_cov_0;
    // cov[14]          // unchanged

    // orientation cov, (roll, pitch, heading) --> (heading, pitch, roll)
    double tmp_cov_21 = cov[21];
    cov[21] = cov[35];
    // cov[28]          // unchanged
    cov[35] = tmp_cov_21;

}

// Conversion of Euler angles (heading, pitch, roll) from NED to
// quaternion in ENU/ ROS (x,y,z,w)
//  yields x-axis as forward axis, for heading = 0 poiting upward to north (y-axis)
void eulerNED2quatENU(double heading, double pitch, double roll,  geometry_msgs::Quaternion& quat)
{
    tf2::Quaternion q_eb;
    tf2::Matrix3x3 m_eb;

    m_eb.setEulerYPR(-heading + M_PI_2, -pitch, roll);
    m_eb.getRotation(q_eb);

    tf2::convert(q_eb, quat);
}

//// Conversion of Euler angles (heading, pitch, roll) from NED to
//// quaternion in ENU/ ROS (x,y,z,w)
//// --- not used ---
////  yields y-axis as forward axis, for heading = 0 poiting upward to north (y-axis)
//void eulerNED2quatENU_ver2(double heading, double pitch, double roll,  geometry_msgs::Quaternion& quat)
//{
//    tf2::Quaternion q_eb;
//    tf2::Matrix3x3 m_eb;
//
//    m_eb.setEulerYPR(heading, pitch, roll);
//    m_eb.getRotation(q_eb);
//
//    // NED to ENU
//    double tmp_qx = q_eb.getX();
//    q_eb.setX(q_eb.getY());
//    q_eb.setY(tmp_qx);
//    q_eb.setZ(-q_eb.getZ());
//
//    tf2::convert(q_eb, quat);
//}

// convert position in NED and Euler angles in NED to pose in ENU
void convertNED2Pose(double pos_N, double pos_E, double pos_D, double heading, double pitch, double roll,
                        geometry_msgs::Pose& pose)
{
    // convert position from NED to ENU: (x,y,z) --> (y,x,-z)
    pose.position.x = pos_E;
    pose.position.y = pos_N;
    pose.position.z = (-1.)*pos_D;

    // convert orientation from Euler angles NED to quaternion in ENU
    eulerNED2quatENU(heading, pitch, roll, pose.orientation);
}

#endif // COMMON_ROS_H

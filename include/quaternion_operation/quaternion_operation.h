// Copyright (c) 2019 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef QUATERNION_OPERATION__QUATERNION_OPERATION_H_
#define QUATERNION_OPERATION__QUATERNION_OPERATION_H_

/**
 * @file quaternion_operation.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definitions of quaternion operation
 * @version 0.1
 * @date 2019-04-21
 *
 * @copyright Copyright (c) 2019
 *
 */

// headers in ROS
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief + Operator overload for geometry_msgs::msg::Quaternion (Addition)
 *
 * @param quat1
 * @param quat2
 * @return geometry_msgs::msg::Quaternion result of Addition
 */
geometry_msgs::msg::Quaternion operator+(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);

/**
 * @brief * Operator overload for geometry_msgs::msg::Quaternion (Multiplication)
 *
 * @param quat1
 * @param quat2
 * @return geometry_msgs::msg::Quaternion result of Multiplication
 */
geometry_msgs::msg::Quaternion operator*(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);

/**
 * @brief namespace of quaternion_operation ROS package
 *
 */
namespace quaternion_operation
{
/**
 * @brief convert Euler angles to Quaternion
 *
 * @param euler Euler angles
 * @return geometry_msgs::msg::Quaternion Quaternion
 */
geometry_msgs::msg::Quaternion convertEulerAngleToQuaternion(geometry_msgs::msg::Vector3 euler);

/**
 * @brief Get the Rotation Matrix from geometry_msgs::msg::Quaternion
 *
 * @param quat input geometry_msgs::msg::Quaternion
 * @return Eigen::Matrix3d get 3x3 Rotation Matrix
 */
Eigen::Matrix3d getRotationMatrix(geometry_msgs::msg::Quaternion quat);

/**
 * @brief convert Quaternion to the Euler angle
 *
 * @param quat Quaternion
 * @return geometry_msgs::msg::Vector3 euler angle
 */
geometry_msgs::msg::Vector3 convertQuaternionToEulerAngle(geometry_msgs::msg::Quaternion quat);

/**
 * @brief checke 2 double values are equal or not
 *
 * @param a
 * @param b
 * @return true a == b
 * @return false a != b
 */
bool equals(double a, double b);

/**
 * @brief check 2 Quaternion values are equal or not
 *
 * @param quat1
 * @param quat2
 * @return true a == b
 * @return false a != b
 */
bool equals(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);

/**
 * @brief convert geometry_msgs::msg::Quaternion to Eigen::MatrixXd
 *
 * @param quat input Quaternion
 * @return Eigen::MatrixXd converted Eigen Matrix (4,1)
 */
Eigen::MatrixXd convertToEigenMatrix(geometry_msgs::msg::Quaternion quat);

/**
 * @brief get conjugate Quaternion
 *
 * @param quat1 input Quaternion
 * @return geometry_msgs::msg::Quaternion conjugate Quaternion
 */
geometry_msgs::msg::Quaternion conjugate(geometry_msgs::msg::Quaternion quat1);

/**
 * @brief rotate Quaternion
 *
 * @param from from pose orientation
 * @param rotation Rotation quaternion
 * @return geometry_msgs::msg::Quaternion Rotated pose orientation
 */
geometry_msgs::msg::Quaternion rotation(
  geometry_msgs::msg::Quaternion from, geometry_msgs::msg::Quaternion rotation);

/**
 * @brief Get the Rotation from 2 Quaternions
 *
 * @param from from pose orientation
 * @param to to pose orientation
 * @return geometry_msgs::msg::Quaternion Rotation between 2 pose orientation described as Quaternion
 * @sa quaternion_operation::rotation
 */
geometry_msgs::msg::Quaternion getRotation(
  geometry_msgs::msg::Quaternion from, geometry_msgs::msg::Quaternion to);

/**
 * @brief Spherical linear interpolation function for geometry_msgs::msg::Quaternion
 *
 * @param quat1
 * @param quat2
 * @param t parameter for interpolation (if t=0,return==quat1),(if t=1,return==quat2)
 * @return geometry_msgs::msg::Quaternion result of Spherical linear interpolation opertaion
 */
geometry_msgs::msg::Quaternion slerp(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2, double t);
}  // namespace quaternion_operation

#endif  // QUATERNION_OPERATION__QUATERNION_OPERATION_H_

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

/**
 * @file quaternion_operation.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implementation of quaternion operation
 * @version 0.1
 * @date 2019-05-04
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <quaternion_operation/quaternion_operation.h>

#include <limits>

geometry_msgs::msg::Quaternion operator+(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2)
{
  geometry_msgs::msg::Quaternion ret;
  ret.x = quat1.x + quat2.x;
  ret.y = quat1.y + quat2.y;
  ret.z = quat1.z + quat2.z;
  ret.w = quat1.w + quat2.w;
  return ret;
}

geometry_msgs::msg::Quaternion operator*(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2)
{
  geometry_msgs::msg::Quaternion ret;
  ret.x = quat1.w * quat2.x - quat1.z * quat2.y + quat1.y * quat2.z + quat1.x * quat2.w;
  ret.y = quat1.z * quat2.x + quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w;
  ret.z = -quat1.y * quat2.x + quat1.x * quat2.y + quat1.w * quat2.z + quat1.z * quat2.w;
  ret.w = -quat1.x * quat2.x - quat1.y * quat2.y - quat1.z * quat2.z + quat1.w * quat2.w;
  return ret;
}

namespace quaternion_operation
{
geometry_msgs::msg::Quaternion convertEulerAngleToQuaternion(geometry_msgs::msg::Vector3 euler)
{
  geometry_msgs::msg::Quaternion ret;
  double roll = euler.x;
  double pitch = euler.y;
  double yaw = euler.z;
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(roll, pitch, yaw);
  ret.x = tf_quat.x();
  ret.y = tf_quat.y();
  ret.z = tf_quat.z();
  ret.w = tf_quat.w();
  /*
  double roll = euler.x;
  double pitch = euler.y;
  double yaw = euler.z;
  Eigen:: Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  geometry_msgs::msg::Quaternion ret;
  ret.x = quat.x();
  ret.y = quat.y();
  ret.z = quat.z();
  ret.w = quat.w();
  */
  return ret;
}

Eigen::Matrix3d getRotationMatrix(geometry_msgs::msg::Quaternion quat)
{
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;
  Eigen::Matrix3d ret(3, 3);
  ret << x * x - y * y - z * z + w * w, 2 * (x * y - z * w), 2 * (z * x + w * y),
    2 * (x * y + z * w), -x * x + y * y - z * z + w * w, 2 * (y * z - x * w), 2 * (z * x - w * y),
    2 * (y * z + w * x), -x * x - y * y + z * z + w * w;
  return ret;
}

geometry_msgs::msg::Vector3 convertQuaternionToEulerAngle(geometry_msgs::msg::Quaternion quat)
{
  geometry_msgs::msg::Vector3 ret;
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 mat(tf_quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  ret.x = roll;
  ret.y = pitch;
  ret.z = yaw;
  /*
  Eigen::Matrix3d m = getRotationMatrix(quat);
  Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);
  ret.x = ea(0);
  ret.y = ea(1);
  ret.z = ea(2);
  */
  return ret;
}

bool equals(double a, double b)
{
  if (fabs(a - b) < DBL_EPSILON) {
    return true;
  }
  return false;
}

bool equals(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2)
{
  if (
    equals(quat1.x, quat2.x) && equals(quat1.y, quat2.y) && equals(quat1.z, quat2.z) &&
    equals(quat1.w, quat2.w)) {
    return true;
  }
  return false;
}

Eigen::MatrixXd convertToEigenMatrix(geometry_msgs::msg::Quaternion quat)
{
  Eigen::MatrixXd ret(4, 1);
  ret << quat.x, quat.y, quat.z, quat.w;
  return ret;
}

geometry_msgs::msg::Quaternion conjugate(geometry_msgs::msg::Quaternion quat1)
{
  quat1.x = quat1.x * -1;
  quat1.y = quat1.y * -1;
  quat1.z = quat1.z * -1;
  // quat1.w = quat1.w * -1;
  return quat1;
}

geometry_msgs::msg::Quaternion rotation(
  geometry_msgs::msg::Quaternion from, geometry_msgs::msg::Quaternion rotation)
{
  return from * rotation;
}

geometry_msgs::msg::Quaternion getRotation(
  geometry_msgs::msg::Quaternion from, geometry_msgs::msg::Quaternion to)
{
  geometry_msgs::msg::Quaternion ans;
  ans = conjugate(from) * to;
  return ans;
}

geometry_msgs::msg::Quaternion slerp(
  geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2, double t)
{
  geometry_msgs::msg::Quaternion q;
  double qr = quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
  double ss = 1.0 - qr * qr;
  constexpr double e = std::numeric_limits<double>::epsilon();
  if (std::fabs(ss) <= e) {
    q.w = quat1.w;
    q.x = quat1.x;
    q.y = quat1.y;
    q.z = quat1.z;
    return q;
  } else {
    double sp = std::sqrt(ss);
    double ph = std::acos(qr);
    double pt = ph * t;
    double t1 = std::sin(pt) / sp;
    double t0 = std::sin(ph - pt) / sp;

    q.w = quat1.w * t0 + quat2.w * t1;
    q.x = quat1.x * t0 + quat2.x * t1;
    q.y = quat1.y * t0 + quat2.y * t1;
    q.z = quat1.z * t0 + quat2.z * t1;
    return q;
  }
}
}  // namespace quaternion_operation

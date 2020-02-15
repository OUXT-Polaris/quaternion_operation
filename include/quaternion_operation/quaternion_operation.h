#ifndef QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
#define QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED

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

//headers in ROS
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

//headers in Eigen
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
geometry_msgs::msg::Quaternion operator+(geometry_msgs::msg::Quaternion quat1,geometry_msgs::msg::Quaternion quat2)
{
    geometry_msgs::msg::Quaternion ret;
    ret.x = quat1.x + quat2.x;
    ret.y = quat1.y + quat2.y;
    ret.z = quat1.z + quat2.z;
    ret.w = quat1.w + quat2.w;
    return ret;
}

/**
 * @brief * Operator overload for geometry_msgs::msg::Quaternion (Multiplication)
 * 
 * @param quat1 
 * @param quat2 
 * @return geometry_msgs::msg::Quaternion result of Multiplication
 */
geometry_msgs::msg::Quaternion operator*(geometry_msgs::msg::Quaternion quat1,geometry_msgs::msg::Quaternion quat2)
{
    geometry_msgs::msg::Quaternion ret;
    ret.x =  quat1.w*quat2.x - quat1.z*quat2.y + quat1.y*quat2.z + quat1.x*quat2.w;
    ret.y =  quat1.z*quat2.x + quat1.w*quat2.y - quat1.x*quat2.z + quat1.y*quat2.w;
    ret.z = -quat1.y*quat2.x + quat1.x*quat2.y + quat1.w*quat2.z + quat1.z*quat2.w;
    ret.w = -quat1.x*quat2.x - quat1.y*quat2.y - quat1.z*quat2.z + quat1.w*quat2.w;
    return ret;
}

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
    geometry_msgs::msg::Quaternion convertEulerAngleToQuaternion(geometry_msgs::msg::Vector3 euler)
    {
        geometry_msgs::msg::Quaternion ret;
        double roll = euler.x;
        double pitch = euler.y;
        double yaw = euler.z;
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(roll,pitch,yaw);
        ret.x = tf_quat.x();
        ret.y = tf_quat.y();
        ret.z = tf_quat.z();
        ret.w = tf_quat.w();
        return ret;
    }
    
    /**
     * @brief Get the Rotation Matrix from geometry_msgs::msg::Quaternion 
     * 
     * @param quat input geometry_msgs::msg::Quaternion 
     * @return Eigen::Matrix3d get 3x3 Rotation Matrix
     */
    Eigen::Matrix3d getRotationMatrix(geometry_msgs::msg::Quaternion quat)
    {
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
        Eigen::Matrix3d ret(3,3);
        ret << 
            x*x-y*y-z*z+w*w, 2*(x*y-z*w),      2*(z*x+w*y),
            2*(x*y+z*w),     -x*x+y*y-z*z+w*w, 2*(y*z-x*w),
            2*(z*x-w*y),     2*(y*z+w*x),      -x*x-y*y+z*z+w*w;
        return ret;
    }

    /**
     * @brief convert Quaternion to the Euler angle
     * 
     * @param quat Quaternion
     * @return geometry_msgs::msg::Vector3 euler angle 
     */
    geometry_msgs::msg::Vector3 convertQuaternionToEulerAngle(geometry_msgs::msg::Quaternion quat)
    {
        geometry_msgs::msg::Vector3 ret;
        tf2::Quaternion tf_quat(quat.x,quat.y,quat.z,quat.w);
        tf2::Matrix3x3 mat(tf_quat);
        double roll,pitch,yaw;
        mat.getRPY(roll, pitch, yaw);
        ret.x = roll;
        ret.y = pitch;
        ret.z = yaw;
        return ret;
    }

    /**
     * @brief checke 2 double values are equal or not
     * 
     * @param a 
     * @param b 
     * @return true a == b
     * @return false a != b
     */
    bool equals(double a,double b)
    {
        if (fabs(a - b) < DBL_EPSILON)
        {
            return true;
        }
        return false;
    }

    /**
     * @brief check 2 Quaternion values are equal or not
     * 
     * @param quat1 
     * @param quat2 
     * @return true a == b
     * @return false a != b
     */
    bool equals(geometry_msgs::msg::Quaternion quat1,geometry_msgs::msg::Quaternion quat2)
    {
        if(equals(quat1.x,quat2.x) && equals(quat1.y,quat2.y) && equals(quat1.z,quat2.z) && equals(quat1.w,quat2.w))
        {
            return true;
        }
        return false;
    }

    /**
     * @brief convert geometry_msgs::msg::Quaternion to Eigen::MatrixXd
     * 
     * @param quat input Quaternion
     * @return Eigen::MatrixXd converted Eigen Matrix (4,1)
     */
    Eigen::MatrixXd convertToEigenMatrix(geometry_msgs::msg::Quaternion quat)
    {
        Eigen::MatrixXd ret(4,1);
        ret << quat.x,quat.y,quat.z,quat.w;
        return ret;
    }

    /**
     * @brief get conjugate Quaternion 
     * 
     * @param quat1 input Quaternion 
     * @return geometry_msgs::msg::Quaternion conjugate Quaternion 
     */
    geometry_msgs::msg::Quaternion conjugate(geometry_msgs::msg::Quaternion quat1)
    {
        quat1.x = quat1.x * -1;
        quat1.y = quat1.y * -1;
        quat1.z = quat1.z * -1;
        //quat1.w = quat1.w * -1;
        return quat1;
    }

    /**
     * @brief rotate Quaternion 
     * 
     * @param from from pose orientation
     * @param rotation Rotation quaternion
     * @return geometry_msgs::msg::Quaternion Rotated pose orientation
     */
    geometry_msgs::msg::Quaternion rotation(geometry_msgs::msg::Quaternion from,geometry_msgs::msg::Quaternion rotation)
    {
        return from*rotation;
    }

    /**
     * @brief Get the Rotation from 2 Quaternions 
     * 
     * @param from from pose orientation
     * @param to to pose orientation
     * @return geometry_msgs::msg::Quaternion Rotation between 2 pose orientation described as Quaternion
     * @sa quaternion_operation::rotation
     */
    geometry_msgs::msg::Quaternion getRotation(geometry_msgs::msg::Quaternion from,geometry_msgs::msg::Quaternion to)
    {
        geometry_msgs::msg::Quaternion ans;
        ans = conjugate(from) * to;
        return ans;
    }

    /**
     * @brief Spherical linear interpolation function for geometry_msgs::msg::Quaternion
     * 
     * @param quat1 
     * @param quat2 
     * @param t parameter for interpolation (if t=0,return==quat1),(if t=1,return==quat2)
     * @return geometry_msgs::msg::Quaternion result of Spherical linear interpolation opertaion
     */
    geometry_msgs::msg::Quaternion slerp(geometry_msgs::msg::Quaternion quat1,geometry_msgs::msg::Quaternion quat2,double t)
    {
        geometry_msgs::msg::Quaternion q;
        double qr = quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
        double ss = (double)1.0 - qr * qr;
        if (ss == (double)0.0)
        {
            q.w = quat1.w;
            q.x = quat1.x;
            q.y = quat1.y;
            q.z = quat1.z;
            return q;
        }
        else
        {
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
}

#endif  //QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
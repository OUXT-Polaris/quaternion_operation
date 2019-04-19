#ifndef QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
#define QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED

//headers in ROS
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

//headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

geometry_msgs::Quaternion operator+(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2)
{
    geometry_msgs::Quaternion ret;
    ret.x = quat1.x + quat2.x;
    ret.y = quat1.y + quat2.y;
    ret.z = quat1.z + quat2.z;
    ret.w = quat1.w + quat2.w;
    return ret;
}

geometry_msgs::Quaternion operator*(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2)
{
    geometry_msgs::Quaternion ret;
    ret.x =  quat1.w*quat2.x - quat1.z*quat2.y + quat1.y*quat2.z + quat1.x*quat2.w;
    ret.y =  quat1.z*quat2.x + quat1.w*quat2.y - quat1.x*quat2.z + quat1.y*quat2.w;
    ret.z = -quat1.y*quat2.x + quat1.x*quat2.y + quat1.w*quat2.z + quat1.z*quat2.w;
    ret.w = -quat1.x*quat2.x - quat1.y*quat2.y - quat1.z*quat2.z + quat1.w*quat2.w;
    return ret;
}

namespace quaternion_operation
{
    geometry_msgs::Quaternion roataion(geometry_msgs::Quaternion pose,geometry_msgs::Quaternion rotation)
    {
        return rotation*pose;
    }

    bool equals(double a,double b)
    {
        if (fabs(a - b) < DBL_EPSILON)
        {
            return true;
        }
        return false;
    }

    bool equals(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2)
    {
        if(equals(quat1.x,quat2.x) && equals(quat1.y,quat2.y) && equals(quat1.z,quat2.z) && equals(quat1.w,quat2.w))
        {
            return true;
        }
        return false;
    }

    Eigen::MatrixXd convert(geometry_msgs::Quaternion quat)
    {
        Eigen::MatrixXd ret(4,1);
        ret << quat.x,quat.y,quat.z,quat.w;
        return ret;
    }

    Eigen::MatrixXd getRotationMatrix(geometry_msgs::Quaternion quat)
    {
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
        Eigen::MatrixXd ret(3,3);
        ret << 
            1-2*(y*y+z*z), 2*(x*y+z*w),   2*(z*x-w*y),
            2*(x*y-z*w),   1-2*(z*z+x*x), 2*(y*z-x*w),
            2*(z*x+w*y),   2*(y*z+w*x),   1-2*(x*x+y*y);
        return ret;
    }
}

#endif  //QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
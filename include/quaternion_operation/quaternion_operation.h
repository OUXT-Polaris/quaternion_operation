#ifndef QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
#define QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED

#include <geometry_msgs/Quaternion.h>

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
        if(!equals(quat1.x,quat2.x))
        {
            return false;
        }
        if(!equals(quat1.y,quat2.y))
        {
            return false;
        }
        if(!equals(quat1.z,quat2.z))
        {
            return false;
        }
        if(!equals(quat1.w,quat2.w))
        {
            return false;
        }
        return true;
    }
}

#endif  //QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
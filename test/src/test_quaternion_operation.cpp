//headers in Google Test
#include <gtest/gtest.h>

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <quaternion_operation/quaternion_operation.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    geometry_msgs::Quaternion ans,q1,q2;
    q1.x = 0;
    q1.y = 1;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 1;
    q2.z = 0;
    q2.w = 1;
    ans = q1 + q2;
    EXPECT_EQ(ans.x,0);
    EXPECT_EQ(ans.y,2);
    EXPECT_EQ(ans.z,0);
    EXPECT_EQ(ans.w,2);
}

// Declare another test
TEST(TestSuite, testCase2)
{
    geometry_msgs::Quaternion ans,q1,q2;
    q1.x = 0;
    q1.y = 1;
    q1.z = 0;
    q1.w = 0;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans = q1 * q2;
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
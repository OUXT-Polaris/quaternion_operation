//headers in Google Test
#include <gtest/gtest.h>

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <quaternion_operation/quaternion_operation.h>

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

TEST(TestSuite, testCase3)
{
    geometry_msgs::Quaternion q1;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    Eigen::MatrixXd mat = quaternion_operation::getRotationMatrix(q1);
    EXPECT_FLOAT_EQ(mat(0,0), 1.0);
    EXPECT_FLOAT_EQ(mat(0,1), 0.0);
    EXPECT_FLOAT_EQ(mat(0,2), 0.0);
    EXPECT_FLOAT_EQ(mat(1,0), 0.0);
    EXPECT_FLOAT_EQ(mat(1,1), 1.0);
    EXPECT_FLOAT_EQ(mat(1,2), 0.0);
    EXPECT_FLOAT_EQ(mat(2,0), 0.0);
    EXPECT_FLOAT_EQ(mat(2,1), 0.0);
    EXPECT_FLOAT_EQ(mat(2,2), 1.0);
}

TEST(TestSuite, testCase4)
{
    geometry_msgs::Quaternion q1;
    q1.x = std::sqrt((double)0.5);
    q1.y = 0;
    q1.z = 0;
    q1.w = std::sqrt((double)0.5);
    Eigen::MatrixXd mat = quaternion_operation::getRotationMatrix(q1);
    EXPECT_FLOAT_EQ(mat(0,0),  1.0);
    EXPECT_FLOAT_EQ(mat(0,1),  0.0);
    EXPECT_FLOAT_EQ(mat(0,2),  0.0);
    EXPECT_FLOAT_EQ(mat(1,0),  0.0);
    //EXPECT_FLOAT_EQ(mat(1,1),  0.0);
    EXPECT_FLOAT_EQ(mat(1,2), -1.0);
    EXPECT_FLOAT_EQ(mat(2,0),  0.0);
    EXPECT_FLOAT_EQ(mat(2,1),  1.0);
    //EXPECT_FLOAT_EQ(mat(2,2),  0.0);
}

TEST(TestSuite, testCase5)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans  = quaternion_operation::slerp(q1,q2,0);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

TEST(TestSuite, testCase6)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 1;
    q2.y = 0;
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::slerp(q1,q2,0);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
    ans  = quaternion_operation::slerp(q1,q2,1);
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
}

TEST(TestSuite, testCase7)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 1;
    q2.y = 0;
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 0;
    q2.z = 1;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = std::sqrt((double)0.5);
    q2.y = std::sqrt((double)0.5);
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = std::sqrt((double)0.4);
    q2.y = std::sqrt((double)0.3);
    q2.z = 0;
    q2.w = std::sqrt((double)0.3);
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = std::sqrt((double)0.8);
    q1.y = 0;
    q1.z = 0;
    q1.w = std::sqrt((double)0.2);
    q2.x = std::sqrt((double)0.4);
    q2.y = std::sqrt((double)0.3);
    q2.z = 0;
    q2.w = std::sqrt((double)0.3);
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
}

TEST(TestSuite, testCase8)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 1;
    q1.y = 0;
    q1.z = 0;
    q1.w = 0;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans  = quaternion_operation::rotation(q1,q2);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
//headers in Google Test
#include <gtest/gtest.h>

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <quaternion_operation/quaternion_operation.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    EXPECT_EQ(10, 10);
}

// Declare another test
TEST(TestSuite, testCase2)
{
    EXPECT_EQ(10, 10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
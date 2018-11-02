#include "FrankaUnitTests.cpp"

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "FrankaTestNode");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
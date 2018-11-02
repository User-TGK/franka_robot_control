#include "AL5DLowLevelDriverTests.hpp"

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "AL5DLowLevelDriverTestNode");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
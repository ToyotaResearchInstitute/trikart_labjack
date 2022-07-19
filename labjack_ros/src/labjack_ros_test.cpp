#include "labjack_ros/labjack_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"labjack_ros_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    labjack_ros test(pnh);
    
    return 0;
}
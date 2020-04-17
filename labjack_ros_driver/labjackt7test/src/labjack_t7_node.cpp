#include "labjackt7test/labjack_t7_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"labjack_t7_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    {
        labjack_t7_ros test(pnh);
        bool temp = false;
        pnh.getParam("streaming",temp);
        if(temp)
        {
            ROS_INFO("Parameter for streaming functionality: TRUE");
            // starts stream thread
            if(test.startStream())
            {
                // stop stream waits for thread to join which is only joinable when !ros::ok() or !_pnh->ok()
                test.stopStream();
            }
            else
            {
                ros::shutdown();
            }
        }
        else
        {
            ROS_INFO("Usual Functionality");
            test.startPublishing();
        }
    }    

    return 0;
}
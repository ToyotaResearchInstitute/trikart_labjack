#include "labjack_ros/labjack_ros_driver.h"
#include <string>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <ros/publisher.h>
#include "labjack_msgs/labjack_channel.h" // msg file for normal functionality
#include "labjack_msgs/labjack_stream.h" // msg file for streaming functionality

class labjack_ros
{
private:
    ros::NodeHandle* _pnh;
    ros::Publisher _stream_publisher;
    std::vector<ros::Publisher> _publishers;
    ros::Rate* _loop;

    labjack_msgs::labjack_stream _stream_msg;

    std::string _device_type, _conn_type, _identifier;

    labjack_driver* _driver;
    std::shared_ptr<std::thread> _streamthread;
    bool _str_started;

    // ros related parameters
    int _acqrate,_pubrate,_numchannels;
    bool _verbose,_streaming;
    std::string _stream_pub_topic;
    std::vector<std::string> _pub_topics;

    std::vector<int> _addresses;
    std::vector<std::string> _names;

    bool getParams();
    void publishStream();

public:
    labjack_ros(ros::NodeHandle& pnh);
    ~labjack_ros();

    // default functionality
    void startPublishing();

    // streaming functionality
    bool startStream();
    void stopStream();
};

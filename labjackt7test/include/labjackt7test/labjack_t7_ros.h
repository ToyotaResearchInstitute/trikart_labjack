#include "labjackt7test/labjack_t7_driver.h"
#include <string>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <ros/publisher.h>
#include "labjackt7test/labjackt7test_channel.h" // msg file for normal functionality
#include "labjackt7test/labjackt7test_streaming.h" // msg file for streaming functionality

class labjack_t7_ros
{
private:
    ros::NodeHandle* _pnh;
    ros::Publisher _stream_publisher;
    std::vector<ros::Publisher> _publishers;
    ros::Rate* _loop;

    labjackt7test::labjackt7test_streaming _stream_msg;

    labjack_t7_driver* _driver;
    std::shared_ptr<std::thread> _streamthread;
    bool _str_started;

    // ros related parameters
    int _acqrate,_pubrate,_numchannels;
    bool _verbose,_streaming;
    std::string _stream_pub_topic;
    std::vector<std::string> _pub_topics;

    bool getParams();
    void publishStream();

public:
    labjack_t7_ros(ros::NodeHandle& pnh);
    ~labjack_t7_ros();

    // default functionality
    void startPublishing();

    // streaming functionality
    bool startStream();
    void stopStream();
};

#include "labjackt7test/labjack_t7_ros.h"

labjack_t7_ros::labjack_t7_ros(ros::NodeHandle& pnh):_pnh(&pnh)
{
    if(!getParams())
    {
        ROS_ERROR("Check parameter server!");
        return;
    }
    _loop = new ros::Rate(_pubrate);

    _driver = new labjack_t7_driver(_numchannels,_acqrate,_verbose);
    if(!_driver->checkConnection())
    {
        ROS_ERROR("Unable to find connection for labjack driver. Use verbose to check driver error.");
        return;
    }
    else
    {
        ROS_DEBUG("Connected to device serial number: %f8.0",_driver->getSerialNumber());
    }

    if(_streaming)
    {
        _driver->setScanRate(_acqrate);
        _driver->setStreaming();
        _stream_publisher = _pnh->advertise<labjackt7test_msgs::labjackt7test_streaming>(_stream_pub_topic.c_str(),1,true);
    }
    else
    {
        for(int i=0;i<_numchannels;i++)
        {
            std::string temp = "labjack/channel_"+std::to_string(i);
            _pub_topics.push_back(temp);
            _publishers.push_back(_pnh->advertise<labjackt7test_msgs::labjackt7test_channel>(temp,1,true));
        }
    }
}

labjack_t7_ros::~labjack_t7_ros()
{
    delete _driver;
}

//default functionality
void labjack_t7_ros::startPublishing()
{
    std::vector<double> temp;
    labjackt7test_msgs::labjackt7test_channel msg;
    while(ros::ok() && _pnh->ok())
    {
        // temp.clear();
        msg.header.stamp = ros::Time::now();
        temp = _driver->getCurrentReadings();
        for(int i=0;i<_publishers.size();i++)
        {
            msg.data.data = temp.at(i);
            _publishers.at(i).publish(msg);
        }
        _loop->sleep();
    }
}

//streaming functionality
bool labjack_t7_ros::startStream()
{
    if(_streamthread)
    {
        ROS_DEBUG("Shared pointer already started thread to stream the device.");
        return false;
    }
    if(!_driver->startStream())
    {
        ROS_DEBUG("Unable to start stream with device.");
        return false;
    }
    _streamthread.reset(new std::thread(&labjack_t7_ros::publishStream,this));
    return true;
}

void labjack_t7_ros::stopStream()
{
    do{
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    while(!_streamthread->joinable());
    _streamthread->join();
    _streamthread.reset();
    _driver->stopStream();
}

void labjack_t7_ros::publishStream()
{
    int dat_size = _driver->getDataSize();
    std::vector<double> aData;
    if(_verbose)
    {
        ROS_INFO("Data size: %d",dat_size);
    }
    while(ros::ok() && _pnh->ok())
    {
        aData.clear();
        double temp[dat_size];
        _driver->acquireReadings(temp); // FIX HERE - not grabbing the data!
        _stream_msg.header.stamp = ros::Time::now();
        for(int i=0;i<dat_size;i++)
        {
            aData.push_back(temp[i]);
        }
        _stream_msg.stream_data.data = aData;
        _stream_publisher.publish(_stream_msg);
        _stream_msg.old_header = _stream_msg.header;
        _loop->sleep();
    }
}

//private functions
bool labjack_t7_ros::getParams()
{
    bool temp = true;
    if(!_pnh->getParam("pubrate",_pubrate))
    {
        temp = false;
        ROS_DEBUG("Unable to find parameter for publish rate of topics");
    }
    if(!_pnh->getParam("numchannels",_numchannels))
    {
        _numchannels = 8;
        ROS_DEBUG("Unable to find parameter for number of channels of topics. Default of 8 is used.");
    }
    if(!_pnh->getParam("acquisitionrate",_acqrate))
    {
        _acqrate = 5000;
        ROS_DEBUG("Unable to find parameter for acquisition rate for device. Default from driver of 5000 is used.");
    }
    if(!_pnh->getParam("verbose",_verbose))
    {
        _verbose = false;
        ROS_DEBUG("Unable to find parameter for verbosity. Default of FALSE is used.");
    }
    if(!_pnh->getParam("streaming",_streaming))
    {
        _streaming = false;
        ROS_DEBUG("Unable to find parameter for streaming from device. Default of FALSE is used.");
    }
    if(_streaming)
    {
        if(!_pnh->getParam("streampubtopic",_stream_pub_topic))
        {
            ROS_DEBUG("Unable to find parameter for streaming publishing topic name.");
            temp = false;
        }
    }
    return temp;
}
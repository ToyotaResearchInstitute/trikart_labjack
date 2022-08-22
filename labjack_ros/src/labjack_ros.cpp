#include "labjack_ros/labjack_ros.h"

labjack_ros::labjack_ros(ros::NodeHandle& pnh):_pnh(&pnh)
{
    _driver = new labjack_driver();
    
    if(!getParams())
    {
        ROS_ERROR("Check parameter server!");
        return;
    }
    _loop = new ros::Rate(_pubrate);

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
        _stream_publisher = _pnh->advertise<labjack_msgs::labjack_stream>(_stream_pub_topic.c_str(),1,true);
    }
    else
    {
        if (_driver->checkChannelNames())
        {
            _names = _driver->getNamesList();
            
            for(std::vector<std::string>::const_iterator it=_names.cbegin();it!=_names.cend();it++)
            {
                std::string temp = "labjack/channel_"+*it;
                _pub_topics.push_back(temp);
                _publishers.push_back(_pnh->advertise<labjack_msgs::labjack_channel>(temp,1,true));
            }
        }
        else
        {
            _addresses = _driver->getAddressList();

            for(std::vector<int>::const_iterator it=_addresses.cbegin();it!=_addresses.cend();it++)
            {
                std::string temp = "labjack/channel_addr_"+std::to_string(*it);
                _pub_topics.push_back(temp);
                _publishers.push_back(_pnh->advertise<labjack_msgs::labjack_channel>(temp,1,true));
            }
        }
    }
    
}

labjack_ros::~labjack_ros()
{
    delete _driver;
}

//default functionality
void labjack_ros::startPublishing()
{
    std::vector<double> temp;
    labjack_msgs::labjack_channel msg;
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
bool labjack_ros::startStream()
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
    _streamthread.reset(new std::thread(&labjack_ros::publishStream,this));
    return true;
}

void labjack_ros::stopStream()
{
    do{
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    while(!_streamthread->joinable());
    _streamthread->join();
    _streamthread.reset();
    _driver->stopStream();
}

void labjack_ros::publishStream()
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
        _driver->acquireReadings(temp);
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
bool labjack_ros::getParams()
{
    bool temp = true;
    if(!_pnh->getParam("pubrate",_pubrate))
    {
        temp = false;
        ROS_DEBUG("Unable to find parameter for publish rate of topics");
    }
    
    if(!_pnh->getParam("acquisitionrate",_acqrate))
    {
        _acqrate = 5000;
        ROS_DEBUG("Unable to find parameter for acquisition rate for device. Default from driver of 5000 is used.");
    }
    
    if( !_pnh->getParam("use_channel_names",_use_channel_names) ||
        !_pnh->getParam("streaming",_streaming) || 
        !_pnh->getParam("verbose",_verbose))
    {
        ROS_DEBUG("Unable to find a necessary streaming parameter (Use Channel Names, Streaming, or Verbose).");
    }
    else
    {
        _driver->setStreamParams(_use_channel_names, _streaming, _verbose);
    }
    
    if(_streaming)
    {
        if(!_pnh->getParam("streampubtopic",_stream_pub_topic))
        {
            ROS_DEBUG("Unable to find parameter for streaming publishing topic name.");
            temp = false;
        }
    }
    if(!_pnh->getParam("channel_names",_names))
    {
        ROS_DEBUG("Unable to find parameter for channel names.");
    }
    else
    {
        _driver->setNamesList(_names);
    }
    if(!_pnh->getParam("channel_addresses",_addresses))
    {
         ROS_DEBUG("Unable to find parameter for channel addresses.");
    }
    else
    {
        _driver->setAddressList(_addresses);           
    }
    
    if( !_pnh->getParam("device_type",_device_type) || 
        !_pnh->getParam("conn_type",_conn_type) || 
        !_pnh->getParam("identifier",_identifier))

    {
        ROS_DEBUG("Unable to find a necessary device paramter (Device Type, Connection Type, or Identifier");
    }
    else
    {
        _driver->setDeviceParams(_device_type, _conn_type,_identifier);
    }

    if(!_pnh->getParam("config_registers", _config_registers_list))
    {
        ROS_DEBUG("No registers defined to write to.");
    }
    else
    {
        XmlRpc::XmlRpcValue sublist;
        std::string address;
        int reg_value;

        for (int i = 0; i < _config_registers_list.size(); i++)
        {
            sublist = _config_registers_list[i];
            address = static_cast<std::string>(sublist["address"]);
            reg_value = static_cast<int>(sublist["value"]);

            _driver->setConfigRegister(address, reg_value);
        }
    }

    return temp;
}
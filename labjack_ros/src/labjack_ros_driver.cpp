#include "labjack_ros/labjack_ros_driver.h"

labjack_driver::labjack_driver(int chan_num)
{
    // input user chosen parameters
    _num_channel = chan_num;

    // set boolean flags to default value
    _dev_found = openConnection();
        
    _acquisition = false;
}

labjack_driver::~labjack_driver()
{
    if(!_dev_found)
    {
        return;
    }

    // force stop acquisition if streaming
    if(_streaming && _acquisition)
    {
        stopStream();   // stops stream
        int numtemp = _num_channel*_str_param._scan_rate/2;
        double temp[numtemp];
        acquireReadings(temp);  // clears buffer
        if(_verbose)
        {
            std::cout << "Final readings: ";
            for(int i=0;i<numtemp;i++)
            {
                std::cout << temp[i] << ", ";
            }
        }
    }

    // force close connection
    do{
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        closeConnection();
        if(checkConnection())
        {
            if(_verbose)
            {
                std::cout << "Connection still exist. Trying closing connection in 500ms." << std::endl;
            }
        }
    }
    while(checkConnection());
}

void labjack_driver::setAddressList(std::vector<int> add_list)
{
    _addresses.clear();
    _addresses = add_list;
}

void labjack_driver::setNamesList(std::vector<std::string> name_list)
{
    _names.clear();
    _names = name_list;
}

std::vector<int>  labjack_driver::getAddressList()
{
    return _addresses;
}

std::vector<std::string> labjack_driver::getNamesList()
{
    return _names;
}

void labjack_driver::setDeviceParams(std::string dev_type, std::string conn_type, std::string identifier)
{
    _device_type = dev_type;
    _comms_type = conn_type;
    _identifier = identifier;
}

void labjack_driver::setStreamParams(bool use_channel_names, bool streaming, bool verbose)
{
    _use_channel_names = use_channel_names;
    _streaming = streaming;
    _verbose = verbose;
}

void labjack_driver::setConfigRegister(std::string address, int reg_value)
{
        std::cout << "Write to " << address << ", value " << reg_value << std::endl;
        //LJM_eWriteName takes a double as the value to write
        _err_code = LJM_eWriteName(_device_handle, address.c_str(), (double) reg_value);
}

double labjack_driver::getSerialNumber()
{
    if(!_dev_found)
    {
        if(_verbose)
        {
            std::cout << "No device connected." << std::endl;
        }
        return 0;
    }
    else
    {
        return _serial_number;
    }
}

bool labjack_driver::checkConnection()
{
    return _dev_found;
}

bool labjack_driver::checkStreaming()
{
    return _streaming;
}

bool labjack_driver::checkChannelNames()
{
    return _use_channel_names;
}


// default functionality
std::vector<double> labjack_driver::getCurrentReadings()
{
    std::vector<double> temp;
    double tempval;

    if (_use_channel_names)
    {
        for(std::vector<std::string>::const_iterator it=_names.begin();it!=_names.end();it++)
        {
            std::cout << *it << std::endl;

            _err_code = LJM_eReadName(_device_handle,it->data(),&tempval);
            temp.push_back(tempval);
            std::cout << tempval << std::endl;
        }
        return temp;
    }
    else
    {
        for(std::vector<int>::iterator it=_addresses.begin();it!=_addresses.end();it++)
        {
            _err_code = LJM_eReadAddress(_device_handle,*it,LJM_INT32,&tempval);
            temp.push_back(tempval);
        }
        return temp;
    }
}


// streaming functionality
void labjack_driver::setStreaming()
{
    if(_use_channel_names)
    {
        if(_verbose)
        {
            std::cout << "Streaming only works with channel addresses, not channel names." << std::endl;
        }
        return;
    }    
    if(_streaming)
    {
        if(_verbose)
        {
            std::cout << "Driver already set to stream data." << std::endl;
        }
        return;
    }
    _streaming = true;
    if(_verbose)
    {
        std::cout << "Setting driver to stream data from addresses: ";
        for(std::vector<int>::iterator it=_addresses.begin();it!=_addresses.end();it++)
        {
            std::cout << *it << ", ";
        }
        std::cout << std::endl << "Driver streaming scan rate: " << _str_param._scan_rate << " Hz" << std::endl;
}
}

void labjack_driver::unsetStreaming()
{
    if(!_streaming)
    {
        if(_verbose)
        {
            std::cout << "Driver already in default functionality." << std::endl;
        }
        return;
    }
    _streaming = false;
    if(_verbose)
    {
        std::cout << "Unset driver to stream data from device. Default functionality is now on." << std::endl;
    }
}

void labjack_driver::setScanRate(double scanrate)
{
    _str_param._scan_rate = scanrate;
    _str_param._scans_per_read = scanrate/_num_channel;
}

double labjack_driver::getScanRate()
{
    return _str_param._scan_rate;
}

int labjack_driver::getDataSize()
{
    return _str_param._scans_per_read*_num_channel;
}

bool labjack_driver::startStream()
{
    if(_use_channel_names)
    {
        if(_verbose)
        {
            std::cout << "Streaming only works with channel addresses, not channel names." << std::endl;
        }
        return false;
    }    
    if(_acquisition)
    {
        if(_verbose)
        {
            std::cout << "Stream already started." << std::endl;
        }
        return false;
    }

    _acquisition = true;
    int scanList[_num_channel];
    std::copy(_addresses.begin(),_addresses.end(),scanList);
    if(_verbose)
    {
        std::cout << "Device Handle: " << _device_handle << "; Scan per read: " << _str_param._scans_per_read << "; Size of addresses: " << _addresses.size() << "; Scan rate: " << _str_param._scan_rate << std::endl;
        std::cout << "Setting streaming settings as per error code 2942 - for USB connection." << std::endl;
    }
    // sets stream settling us to 0
    _err_code = LJM_eWriteAddress(_device_handle,4008,LJM_FLOAT32,0);
    if(_verbose)
    {
        std::cout << "Set stream settling us to 0. Error code: " << _err_code << std::endl;
    }
    // sets stream resolution index to 0
    _err_code = LJM_eWriteAddress(_device_handle,4010,LJM_UINT32,0);
    if(_verbose)
    {
        std::cout << "Set stream resolution index to 0. Error code: " << _err_code << std::endl;
    }

    _err_code = LJM_eStreamStart(_device_handle,_str_param._scans_per_read,_addresses.size(),scanList,&_str_param._scan_rate);
    if(_err_code != 0)
    {
        _acquisition = false;
        if(_verbose)
        {
            std::cout << "Error in starting stream. Stream not started. Error code: " << _err_code << std::endl;
        }
        return false;
    }
    return true;
}

void labjack_driver::stopStream()
{
    if(!_acquisition)
    {
        if(_verbose)
        {
            std::cout << "Stream already stopped for device." << std::endl;
        }
        return;
    }
    _err_code = LJM_eStreamStop(_device_handle);
    _acquisition = false;
    if(_err_code != 0)
    {
        if(_verbose)
        {
            std::cout << "Error in stopping stream. Error code: " << _err_code << std::endl;
        }
        _acquisition = true;
    }
    else
    {
        if(_verbose)
        {
            std::cout << "Stream stopped for device serial number: " << getSerialNumber() << std::endl;
        }
    }
}

bool labjack_driver::acquireReadings(double *data)
{
    if(!_acquisition)
    {
        if(_verbose)
        {
            if(!_streaming)
            {
                std::cout << "Streaming functionality not set for driver." << std::endl;
            }
            else
            {
                std::cout << "Can't acquire readings. No stream has been started." << std::endl;
            }
        }
        return false;
    }
    _err_code = LJM_eStreamRead(_device_handle,data,&_str_param._devscan_backlog,&_str_param._ljmscan_backlog);
    if(_err_code != 0)
    {
        if(_verbose)
        {
            std::cout << "Error in stream read. Error code " << _err_code << std::endl;
        }
        return false;
    }
    else
    {
        if(_verbose)
        {
            std::cout << "Stream read. Error code: " << _err_code << "; Device backlog: " << _str_param._devscan_backlog << "; LJM backlog: " << _str_param._ljmscan_backlog << std::endl;
        }
        return true;
    }
}


// Private functions
bool labjack_driver::openConnection()
{
    try
    {
        _err_code = LJM_OpenS(_device_type.c_str(),
                              _comms_type.c_str(),
                              _identifier.c_str(),
                              &_device_handle);        
    }
    catch(const std::exception& e)
    {
        if(_verbose)
        {
            std::cerr << e.what() << '\n';
        }
    }
    if(_err_code !=0)
    {
        if(_verbose)
        {
            std::cout << "Error in opening connection with device." << std::endl;
        }
        return false;
    }

    try{
        if(_verbose)
        {
            std::cout << "Reading serial number" << std::endl;
        }
        _err_code = LJM_eReadName(_device_handle,"SERIAL_NUMBER",&_serial_number);
        if(_err_code != 0)
        {
            if(_verbose)
            {
                std::cout << "Error in reading serial number!" << std::endl;
            }
            LJM_CloseAll();
            return false;
        }        
    }
    catch(const std::exception& ex)
    {
        if(_verbose)
        {
            std::cerr << ex.what() << std::endl;
        }
        LJM_CloseAll();
        return false;
    }
    return true;
}

void labjack_driver::closeConnection()
{
    if(!_dev_found)
    {
        return;
    }

    if(_verbose)
    {
        std::cout << "Closing connection to device serial number: " << _serial_number << std::endl;
    }
    _err_code = LJM_Close(_device_handle);
    if(_err_code !=0 && _verbose)
    {
        std::cout << "Error in closing connection with device" << std::endl;
    }
    _dev_found = false;
}
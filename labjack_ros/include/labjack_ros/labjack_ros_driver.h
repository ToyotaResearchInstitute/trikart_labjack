#include <LabJackM.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <string>

class labjack_driver
{
private:
    int _num_channel,_err_code,_device_handle;
    bool _verbose,_dev_found,_streaming,_use_channel_names;
    double _serial_number;
    std::string _device_type,_comms_type,_identifier;

    //streaming specific parameters for device
    // struct stream_param{
    //     int _scans_per_read,_devscan_backlog,_ljmscan_backlog;
    //     double _scan_rate;
    // }_str_param;


    int _scans_per_read,_devscan_backlog,_ljmscan_backlog;
    double _scan_rate;


    //parameters relating to operation of driver
    std::vector<int> _addresses;
    std::vector<std::string> _names;

    // object for streaming functionality
    bool _acquisition;
    
    // private functions
    bool openConnection();  // opens connection to device
    void closeConnection(); // closes all connections

public:
    labjack_driver();
    ~labjack_driver();

    // setter
    void setAddressList(std::vector<int> add_list);
    void setNamesList(std::vector<std::string> name_list);
    std::vector<int>  getAddressList();
    std::vector<std::string> getNamesList();

    void setDeviceParams(std::string dev_type, std::string conn_type, std::string identifier);
    void setStreamParams(bool use_channel_names, bool streaming, bool verbose);
    void setConfigRegister(std::string address, int reg_value);

    // checks and getters
    double getSerialNumber(); // getter for serial number of device
    bool checkConnection(); //check whether connection to device is valid
    bool checkStreaming(); // check if driver is setup for streaming
    bool checkChannelNames(); //check if driver is using channel names or addresses

    // default functionality
    std::vector<double> getCurrentReadings();   // get current acquisition reading for wrapper

    // streaming functionality
    void setStreaming(); // setup streaming parameters
    void unsetStreaming(); // clear streaming parameters
    void setScanParams(double scanrate, int scans_per_read); // sets scanning rate and scans per read
    double getScanRate(); // getter for scan rate
    int getDataSize(); // getter for datasize for readings from stream

    bool startStream(); // starts the streaming thread
    void stopStream(); // stops the streaming thread
    bool acquireReadings(double *data); // acquire all buffer readings
};

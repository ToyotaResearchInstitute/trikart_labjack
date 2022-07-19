#include <LabJackM.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>

class labjack_driver
{
private:
    //default parameters for device
    struct default_param{
        int _device_type,_comms_type,_num_channel,_err_code,_device_handle,_acqrate;
        bool _verbose,_dev_found,_streaming;
        double _serial_number;
        std::string _identifier;
    }_def_param;

    //streaming specific parameters for device
    struct stream_param{
        int _scans_per_read,_devscan_backlog,_ljmscan_backlog;
        double _scan_rate;
    }_str_param;

    //parameters relating to operation of driver
    std::vector<int> _addresses;

    // object for streaming functionality
    bool _acquisition;
    
    // private functions
    bool openConnection();  // opens connection to device
    void closeConnection(); // closes all connections

public:
    labjack_driver(int chan_num = 8,int acq_rate = 5000,bool verbose = false,std::string identifier="LJM_idANY",double serial_num = 0,int dev_type = 0,int comm_type = 1);
    ~labjack_driver();

    // setter
    void setAddressList(std::vector<int> add_list);

    // checks and getters
    double getSerialNumber(); // getter for serial number of device
    bool checkConnection(); //check whether connection to device is valid
    bool checkStreaming(); // check if driver is setup for streaming

    // default functionality
    std::vector<double> getCurrentReadings();   // get current acquisition reading for wrapper

    // streaming functionality
    void setStreaming(); // setup streaming parameters
    void unsetStreaming(); // clear streaming parameters
    void setScanRate(double scanrate); // sets scanning rate
    double getScanRate(); // getter for scan rate
    int getDataSize(); // getter for datasize for readings from stream

    bool startStream(); // starts the streaming thread
    void stopStream(); // stops the streaming thread
    bool acquireReadings(double *data); // acquire all buffer readings
};

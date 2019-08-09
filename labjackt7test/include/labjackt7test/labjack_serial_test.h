// #include <stdio.h>
#include <iostream>
#include <LabJackM.h>

class labjack_serial_test
{
private:
    // constructor arguments
    double _serial_number;
    int _device_type,_comms_type;

    // error code of functions, device handle
    int _err_code,_device_handle;
public:
    labjack_serial_test(double serial_num = 0,int dev_type = 7,int comm_type = 1); //default serial num is for ANY, device type is T7 series, comms type is USB
    ~labjack_serial_test();
};

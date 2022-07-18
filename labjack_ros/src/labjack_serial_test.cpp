#include "labjack_ros/labjack_serial_test.h"


labjack_serial_test::labjack_serial_test(double serial_num,int dev_type,int comm_type):_serial_number(serial_num),_device_type(dev_type),_comms_type(comm_type)
{
    std::cout.precision(15);
    std::cout << "Opening LabJack device" << std::endl;
    _err_code = LJM_Open(_device_type,_comms_type,"LJM_idANY", &_device_handle);
    if(_err_code != 0)
    {
        std::cout << "Error in opening LabJack T7" << std::endl;
        return;
    }

    if(_serial_number == 0)
    {
        _err_code = LJM_eReadName(_device_handle,"SERIAL_NUMBER",&_serial_number);
    }
    if(_err_code == 0)
    {
        std::cout << "Serial number of LabJack device: " << _serial_number << std::endl;
    }
    else
    {
        std::cout << "Error in reading serial number" << std::endl;
    }
    
    do
    {
        _err_code = LJM_Close(_device_handle);
    }
    while(_err_code != 0);
    std::cout << "Closed LabJack device connection" << std::endl;
}

labjack_serial_test::~labjack_serial_test()
{
}

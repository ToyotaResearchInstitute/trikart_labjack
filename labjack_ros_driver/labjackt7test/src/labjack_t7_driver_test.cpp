#include "labjackt7test/labjack_t7_driver.h"

int main(int argc, char** argv)
{
    std::cout << std::setprecision(8);
    std::cout << "Starting labjack_t7_driver test" << std::endl;
    std::cout << "Default values used: any serial number, t7 series device, USB comms type, acq rate of 1000Hz, 8 channels" << std::endl;
    labjack_t7_driver test;

    // if(test.openConnection())
    // {
    //     std::cout << "Device connected" << std::endl;
    //     std::cout << "Serial Number connected: " << test.getSerialNumber() << std::endl;
    // }
    // else
    // {
    //     std::cout << "Unable to connect to device! Closing test. Check that the labjack is connected to this device." << std::endl;
    //     return -1;
    // }
    
    // std::cout << "Sleep for 1 second" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // std::cout << "Testing acquisition start" << std::endl;
    // try
    // {
    //     test.startAcquisition();
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }

    // std::cout << "Sleep for 1 second" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // std::cout << "Test obtaining reading" << std::endl;
    // try
    // {
    //     std::vector<std::pair<double,std::string> > test_reading;

    //     for(int i = 0; i < 10; i++)
    //     {
    //         std::cout << "Test reading sample " << i << std::endl;
    //         test_reading = test.getCurrentReadings();
    //         for(std::vector<std::pair<double,std::string> >::iterator it=test_reading.begin();it!=test_reading.end();it++)
    //         {
    //             std::cout << "Channel " << it->second.c_str() << ": " << it->first << std::endl;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(250));
    //     }
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
    
    // std::cout << "Sleep for 1 second" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // std::cout << "Closing connection" << std::endl;
    // try
    // {
    //     test.stopAcquisition();
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
    

    return 0;
}
Author: Yujun Lai (yujun.lai@student.uts.edu.au)

# Purpose
This is a ros package which is used for the LabJack T7 DAQ. Currently it's used for collecting EMG data using the Delsys 8-Channel Bagnoli System. The default values for the driver are catered towards USB connection for the T7, but it can be changed easily to incorporate other methods (Ethernet, Wifi) and devices in LabJack (T4 series).

# Dependencies
For this to work, you do require the LabJack proprietary libraries LJM which can be obtained at:
https://labjack.com/support/software/api/ljm

Changes are required in the CMakeLists.txt (line 12) to reflect where you have installed the libraries. The default script from Labjack will install them at `/usr/local/lib/include`

### Launch Arguments
```
roslaunch labjackt7test labjack_t7.launch
```

`publishrate` - publishing rate for ros (can go up to ~80Hz for 8 channels for usual functionality, streaming mode usually publishes at ~9Hz since the data is all buffered together)

`channelnumber` - number of channels in the Bagnoli

`acqrate` - acquisition rate of the LabJack. Tested with up to 12000Hz for 8 channels (1.5kHz for all channels) using USB. The device can stream sample a single channel up to 100kHz.

`streampubtopic` - ros publisher topic name for streamed data

`verbose` - verbosity flag. Tons of std::cout in driver for debugging. Default value is false.

`streaming` - flag for streaming mode. Default is false, leading to call response mode (default device mode).

## (Other) Nodes

#### labjackt7test_connection_test
Simple node to test connection by calling for the device's serial number and displaying on cout.

#### labjackt7test_driver_test
Simple node to test device driver by calling 10 samples using call response mode.

#### labjackt7test_node
Node used in launch file. Include functionality for default and streaming mode.

#### labjackt7test_kill
Used for debugging purposes. Calls LJM to close connection from this device to all DAQ devices.

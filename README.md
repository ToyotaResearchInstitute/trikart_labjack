Author: Velin Dimitrov (velin.dimitrov@tri.global)

# Purpose
This is a ROS package forked from the UTS-CAS/labjack_ros_driver repository to support the LabJack devices on the TRIKart platform.

# Dependencies
The package has been successfully tested on a NVIDIA Xavier NX with the LJM Library. The Xavier NX requries the Linux aarch64 version of the LJM Installer and works with Jetpack 4.6.

For this to work, you do require the LabJack proprietary libraries LJM which can be obtained at:
https://labjack.com/support/software/api/ljm

Changes are required in the CMakeLists.txt (line 12) to reflect where you have installed the libraries. The default script from Labjack will install the library header files at `/usr/local/include`

### Launch Arguments
```
roslaunch labjack_ros labjack_ros.launch
```

`publishrate` - publishing rate for ros (can go up to ~80Hz for 8 channels for usual functionality, streaming mode usually publishes at ~9Hz since the data is all buffered together)

`channelnumber` - number of channels in the Bagnoli

`acqrate` - acquisition rate of the LabJack. Tested with up to 12000Hz for 8 channels (1.5kHz for all channels) using USB. The device can stream sample a single channel up to 100kHz.

`streampubtopic` - ros publisher topic name for streamed data

`verbose` - verbosity flag. Tons of std::cout in driver for debugging. Default value is false.

`streaming` - flag for streaming mode. Default is true. If set to false, device will use call response mode.

## (Other) Nodes

#### labjack_ros_connection_test
Simple node to test connection by calling for the device's serial number and displaying on cout.
```
roslaunch labjack_ros labjack_ros_connection_test.launch
```

#### labjack_ros_driver_test
Simple node to test device driver by calling 10 samples using call response mode.

#### labjack_ros_node
Node used in launch file. Include functionality for default and streaming mode.

#### labjack_ros_kill
Used for debugging purposes. Calls LJM to close connection from this device to all DAQ devices.

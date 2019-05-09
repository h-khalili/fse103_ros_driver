
# Variense FSE103 ROS Driver

This project contains a ROS driver for the Variense FSE103 force sensor.

## Installation

### Prerequisites

This package depends on the libusb library and requires its developer version installed:

```
sudo apt-get install libusb-1.0-0-dev
```

### ROS package

Currently the package can be installed by cloning this github repository into your catkin workspace and building the package:
```
    cd ~/catkin_ws/src
    mkdir variense && cd variense
    git clone https://github.com/h-khalili/fse103_ros_driver.git variense_fse103
    cd ~/catkin_ws && catkin_make
```

## Running

### Root access permission

In order to send and receive data over  USB, the user must have read and write access to the USB device. A simple method to achieve this is to gain root access. Don't forget to source your workspace setup file:
```
sudo su
source ~/catkin_ws/devel/setup.bash
```
Alternatively, one could modify udev rules to avoid having to do this every time:
```
cd /etc/udev/rules.d
sudo su
touch fse103.rules 
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="0c21", MODE:="0660", GROUP:="plugdev"' > fse103.rules
udevadm control --reload-rules && udevadm trigger

```
Instead of granting permission to a group (plugdev in this case), the owner can be set directly by replacing ```GROUP:="plugdev"``` with ```OWNER:="<user>"```.

### Command line 

The driver can be run from the command line as shown below where the driver will attempt to connect to the first FSE103 sensor it finds :
```
rosrun variense_fse103 fse103_node
```
By passing serial number of the sensor as a parameter, the driver can be run for that specific sensor. Additional optional parameters can also be passed:
```
rosrun variense_fse103 fse103_node _serial_number:="103EAA8876" _filter_bandwidth:="6"
```
A full list of parameters is provided in the **Parameters** section. With multiple parameters, a more elegant way is to use a launch file to run the driver.

### Launch file

The launch file ```variense_fse103/launch/force_sensors.launch``` shows an example of how to run multiple drivers (or a single driver) for different force sensors from a launch file. The launch file can be run normally:
```
roslaunch variense_fse103 force_sensors.launch
```

## Listing all nodes

In order to find all the Variense FSE103 sensors connected to the system, and their corresponding serial numbers, you can run the following node:
```
rosrun variense_fse103 list_sensors 
```
or optionally adding ```-v``` for a detailed display of the sensors.

Remember that this requires USB access permission! 

## Parameters
This section includes a list of all the parameters that can be used to configure the force sensor driver. These can either be passed using the command line as:
```
_<par_name>:=<par_value>
```
or using the launch file as:
```
<param name="<par_name>" value=<par_value> />
```
where ```<par_name>``` and ```<par_value>``` are the name and value of the parameter.

### Parameter list
- ```serial_number``` (*Default*: ```""```)
The serial number of the specified force sensor. 
- ```rate``` (*Default*: ```"200"```)
The publishing rate of the sensor data in Hz. Note that the maximum sampling frequency of the sensor is just over 210Hz.
- ```filter_bandwidth``` (*Default*: ```"0"```)
The cut-off frequency of the optional lowpass 2<sup>nd</sup> order Butterworth filter. A value of 0 indicates no filtering. The maximum frequency is half the sampling rate ([Nyquist theorem](https://en.wikipedia.org/wiki/Nyquist%E2%80%93Shannon_sampling_theorem)).
- ```sensor_id``` (*Default*: ```""```)
The sensor id that determines the name of the publishing topic (explained in the next section). This can be any string.
- ```init_on_start``` (*Default*: ```"true"```)
This parameter determines whether the force sensor is sent an initialisation command when the node starts. The initialisation resets the current offset to zero.


## Published topic

By default the force sensor measurements are published to:
- ```/force_sensor``` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html))

but with a non-empty ```sensor_id``` parameter, the topic name becomes:
- ```/force_sensor_<sensor_id>```

where ```<sensor_id>``` is the parameter value.
## Available services

Currently, the only available service is sensor initialisation (zeroing the offset). For a node with the default name ```/force_sensor_node```, the service can be called as follows:
```
rosservice call /force_sensor_node/initialise
```
## Node name

The name of the node can be changed through the command line:
```
rosrun variense_fse103 fse103_node __name:="<node_name>"
```
and in the launch file:
```
<node name="<node_name>" pkg="variense_fse103" type="fse103_node">
... </node>
```
where ```<node_name>``` is the specified node name.
## Built with

* [libusb](https://libusb.info/) - C library that provides generic access to USB devices
* [IIR1 -- Realtime C++ filter library](https://github.com/berndporr/iir1) by Bernd Porr

## Authors

* **Hassan H. Khalili** - [h-khalili](https://github.com/h-khalili)

## License

This project is licensed under the GNU General Public License v2.0 - see the [LICENSE](LICENSE) file for details.

## Reusability

The source code of the sensor driver has been developed as header-only libraries which are independent of ROS. The files in the ```include/variense_fse103```  can be reused with generic C++ code. A test program is also included in the ```test``` folder.

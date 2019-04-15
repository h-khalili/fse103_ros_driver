# Variense FSE103 ROS Driver

This project contains a ROS driver for the Variense FSE103 force sensor.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine. 

### Prerequisites

This package depends on the libusb library and requires its developer version installed:

```
sudo apt-get install libusb-1.0-0-dev
```

### Installing

Currently the package can be installed from cloning this github repository:

```
cd ~/catkin_ws/src
mkdir variense && cd variense
git clone https://github.com/h-khalili/fse103_ros_driver.git
cd ~/catkin_ws && catkin_make
```

## Published topics



## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [libusb](https://libusb.info/) - C library that provides generic access to USB devices

## Authors

* **Hassan H. Khalili** - [h-khalili](https://github.com/h-khalili)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments




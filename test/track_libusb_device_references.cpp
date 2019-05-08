#include <iostream>
#include "../include/variense_fse103/libvariense_fse103.h"
using namespace std;

void printdev(libusb_device *dev); //prototype of the function

int main() {
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	libusb_context *ctx = NULL; //a libusb session
	int r; //for return values
	ssize_t cnt; //holding number of devices in list
	r = libusb_init(&ctx); //initialize a library session
	if(r < 0) {
		cout<<"Init Error "<<r<<endl; //there was an error
				return 1;
	}
	libusb_set_debug(ctx, 3); //set verbosity level to 3, as suggested in the documentation
	cnt = libusb_get_device_list(ctx, &devs); //get the list of devices
	if(cnt < 0) {
		cout<<"Get Device Error"<<endl; //there was an error
	}
	cout<<cnt<<" Devices in list."<<endl; //print total number of usb devices
		ssize_t i; //for iterating through the list
	for(i = 0; i < cnt; i++) {
        printf("dev pointer: %p\n", devs[i]);
		std::cout<<"dev refcount: "<< libusb_get_refcnt(devs[i]) <<std::endl;
		printdev(devs[i]); //print specs of this device
	}
    libusb_free_device_list(devs, 1); //free the list, unref the devices in it
    std::cout << "-------------Devices freed ... " << std::endl;
	for(i = 0; i < cnt; i++) {
        printf("dev pointer: %p\n", devs[i]);
		std::cout<<"dev refcount: "<< libusb_get_refcnt(devs[i]) <<std::endl;
	}
    libusb_exit(ctx); //close the session
    return 0;
}

void printdev(libusb_device *dev) {
	libusb_device_descriptor desc;
	int r = libusb_get_device_descriptor(dev, &desc);
	if (r < 0) {
		cout<<"failed to get device descriptor"<<endl;
		return;
	}
	cout<<"Device Class: "<<(int)desc.bDeviceClass<<"  ";
	cout<<"VendorID: "<<desc.idVendor<<"  ";
	cout<<"ProductID: "<<desc.idProduct<<endl;
}

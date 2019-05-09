#include "libusb-1.1.h"
#include <iostream>
#include <libusb-1.0/libusb.h>
using namespace std;

uint16_t vendor_id = 0x16d0;
uint16_t product_id = 0x0c21;

int main(int argc, char *argv[]) {
  libusb_device *
      *devs; // pointer to pointer of device, used to retrieve a list of devices
  libusb_context *ctx = NULL; // a libusb session
  ssize_t r;                  // for return values
  ssize_t cnt;                // holding number of devices in list
  ssize_t i = 0;              // for iterating through the list
  int verbose = 0;

  if (argc > 1 && !strcmp(argv[1], "-v"))
    verbose = 1;

  r = libusb_init(&ctx); // initialize a library session
  if (r < 0) {
    cout << "Init Error " << r << endl; // there was an error
    return r;
  }

  // Get the list of devices
  cnt = libusb_get_device_list_with_vid_pid(ctx, &devs, vendor_id, product_id);
  if (cnt < 0) {
    cout << "Get Device Error" << endl; // there was an error
    return cnt;
  }

  cout << cnt << " Devices in list."
       << endl; // print total number of usb devices
  for (i = 0; devs[i]; ++i) {
    libusb_print_device(devs[i], 0, verbose); // print specs of this device
  }

  libusb_free_device_list(devs, 1); // free the list, unref the devices in it
  libusb_exit(ctx);                 // close the session
  return 0;
}
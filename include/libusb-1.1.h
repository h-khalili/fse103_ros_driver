/*
 * Author: Hassan Hakim Khalili <hassan.hakimkhalili@manchester.ac.uk>
 * 
 * This header file extends the libusb-1.0 library. It includes the 
 * functions defined in testlibusb.c by Nathan Hjelm <hjelmn@mac.ccom>
 * for printing the properties of a USB device. The file is available at:
 * https://github.com/libusb/libusb/tree/master/examples/testlibusb.c
 * 
 * The print_device function was modified to libusb_print_device with a
 * slightly different signature.
 * 
 * The library extension includes functions for retrieving a list of
 * devices by their vendor/product ids and for obtaining a device handle
 * to a device with a specific serial number.
 * 
 * The code should be compatible with C/C++.
 */

#ifndef LIBUSB1_1
#define LIBUSB1_1

#include <stdio.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include <stdlib.h>

#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define snprintf _snprintf
#endif


static void print_endpoint_comp(const struct libusb_ss_endpoint_companion_descriptor *ep_comp)
{
	printf("      USB 3.0 Endpoint Companion:\n");
	printf("        bMaxBurst:        %d\n", ep_comp->bMaxBurst);
	printf("        bmAttributes:     0x%02x\n", ep_comp->bmAttributes);
	printf("        wBytesPerInterval: %d\n", ep_comp->wBytesPerInterval);
}

static void print_endpoint(const struct libusb_endpoint_descriptor *endpoint)
{
	int i, ret;

	printf("      Endpoint:\n");
	printf("        bEndpointAddress: %02xh\n", endpoint->bEndpointAddress);
	printf("        bmAttributes:     %02xh\n", endpoint->bmAttributes);
	printf("        wMaxPacketSize:   %d\n", endpoint->wMaxPacketSize);
	printf("        bInterval:        %d\n", endpoint->bInterval);
	printf("        bRefresh:         %d\n", endpoint->bRefresh);
	printf("        bSynchAddress:    %d\n", endpoint->bSynchAddress);

	for (i = 0; i < endpoint->extra_length;) {
		if (LIBUSB_DT_SS_ENDPOINT_COMPANION == endpoint->extra[i + 1]) {
			struct libusb_ss_endpoint_companion_descriptor *ep_comp;

			ret = libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
			if (LIBUSB_SUCCESS != ret) {
				continue;
			}

			print_endpoint_comp(ep_comp);

			libusb_free_ss_endpoint_companion_descriptor(ep_comp);
		}

		i += endpoint->extra[i];
	}
}

static void print_altsetting(const struct libusb_interface_descriptor *interface)
{
	uint8_t i;

	printf("    Interface:\n");
	printf("      bInterfaceNumber:   %d\n", interface->bInterfaceNumber);
	printf("      bAlternateSetting:  %d\n", interface->bAlternateSetting);
	printf("      bNumEndpoints:      %d\n", interface->bNumEndpoints);
	printf("      bInterfaceClass:    %d\n", interface->bInterfaceClass);
	printf("      bInterfaceSubClass: %d\n", interface->bInterfaceSubClass);
	printf("      bInterfaceProtocol: %d\n", interface->bInterfaceProtocol);
	printf("      iInterface:         %d\n", interface->iInterface);

	for (i = 0; i < interface->bNumEndpoints; i++)
		print_endpoint(&interface->endpoint[i]);
}

static void print_2_0_ext_cap(struct libusb_usb_2_0_extension_descriptor *usb_2_0_ext_cap)
{
	printf("    USB 2.0 Extension Capabilities:\n");
	printf("      bDevCapabilityType: %d\n", usb_2_0_ext_cap->bDevCapabilityType);
	printf("      bmAttributes:       0x%x\n", usb_2_0_ext_cap->bmAttributes);
}

static void print_ss_usb_cap(struct libusb_ss_usb_device_capability_descriptor *ss_usb_cap)
{
	printf("    USB 3.0 Capabilities:\n");
	printf("      bDevCapabilityType: %d\n", ss_usb_cap->bDevCapabilityType);
	printf("      bmAttributes:       0x%x\n", ss_usb_cap->bmAttributes);
	printf("      wSpeedSupported:    0x%x\n", ss_usb_cap->wSpeedSupported);
	printf("      bFunctionalitySupport: %d\n", ss_usb_cap->bFunctionalitySupport);
	printf("      bU1devExitLat:      %d\n", ss_usb_cap->bU1DevExitLat);
	printf("      bU2devExitLat:      %d\n", ss_usb_cap->bU2DevExitLat);
}

static void print_bos(libusb_device_handle *handle)
{
	struct libusb_bos_descriptor *bos;
	int ret;

	ret = libusb_get_bos_descriptor(handle, &bos);
	if (0 > ret) {
		return;
	}

	printf("  Binary Object Store (BOS):\n");
	printf("    wTotalLength:       %d\n", bos->wTotalLength);
	printf("    bNumDeviceCaps:     %d\n", bos->bNumDeviceCaps);

	if(bos->dev_capability[0]->bDevCapabilityType == LIBUSB_BT_USB_2_0_EXTENSION) {

		struct libusb_usb_2_0_extension_descriptor *usb_2_0_extension;
	        ret =  libusb_get_usb_2_0_extension_descriptor(NULL, bos->dev_capability[0],&usb_2_0_extension);
	        if (0 > ret) {
		        return;
	        }

                print_2_0_ext_cap(usb_2_0_extension);
                libusb_free_usb_2_0_extension_descriptor(usb_2_0_extension);
        }

	if(bos->dev_capability[0]->bDevCapabilityType == LIBUSB_BT_SS_USB_DEVICE_CAPABILITY) {

	        struct libusb_ss_usb_device_capability_descriptor *dev_cap;
		ret = libusb_get_ss_usb_device_capability_descriptor(NULL, bos->dev_capability[0],&dev_cap);
	        if (0 > ret) {
		        return;
	        }

	        print_ss_usb_cap(dev_cap);
	        libusb_free_ss_usb_device_capability_descriptor(dev_cap);
        }

	libusb_free_bos_descriptor(bos);
}

static void print_interface(const struct libusb_interface *interface)
{
	int i;

	for (i = 0; i < interface->num_altsetting; i++)
		print_altsetting(&interface->altsetting[i]);
}

static void print_configuration(struct libusb_config_descriptor *config)
{
	uint8_t i;

	printf("  Configuration:\n");
	printf("    wTotalLength:         %d\n", config->wTotalLength);
	printf("    bNumInterfaces:       %d\n", config->bNumInterfaces);
	printf("    bConfigurationValue:  %d\n", config->bConfigurationValue);
	printf("    iConfiguration:       %d\n", config->iConfiguration);
	printf("    bmAttributes:         %02xh\n", config->bmAttributes);
	printf("    MaxPower:             %d\n", config->MaxPower);

	for (i = 0; i < config->bNumInterfaces; i++)
		print_interface(&config->interface[i]);
}

int libusb_print_device(libusb_device *dev, int level, int verbose)
{
	struct libusb_device_descriptor desc;
	libusb_device_handle *handle = NULL;
	char description[260];
	unsigned char string[256];
	int ret;
	uint8_t i;

	ret = libusb_get_device_descriptor(dev, &desc);
	if (ret < 0) {
		fprintf(stderr, "failed to get device descriptor");
		return -1;
	}

	ret = libusb_open(dev, &handle);
	if (LIBUSB_SUCCESS == ret) {
		if (desc.iManufacturer) {
			ret = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, string, sizeof(string));
			if (ret > 0)
				snprintf(description, sizeof(description), "%s - ", string);
			else
				snprintf(description, sizeof(description), "%04X - ",
				desc.idVendor);
		}
		else
			snprintf(description, sizeof(description), "%04X - ",
			desc.idVendor);

		if (desc.iProduct) {
			ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct, string, sizeof(string));
			if (ret > 0)
				snprintf(description + strlen(description), sizeof(description) -
				strlen(description), "%s", string);
			else
				snprintf(description + strlen(description), sizeof(description) -
				strlen(description), "%04X", desc.idProduct);
		}
		else
			snprintf(description + strlen(description), sizeof(description) -
			strlen(description), "%04X", desc.idProduct);
	}
	else {
		snprintf(description, sizeof(description), "%04X - %04X",
			desc.idVendor, desc.idProduct);
	}

	printf("%.*sDev (bus %d, device %d): %s\n", level * 2, "                    ",
		libusb_get_bus_number(dev), libusb_get_device_address(dev), description);

	if (handle && verbose) {
		if (desc.iSerialNumber) {
			ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, string, sizeof(string));
			if (ret > 0)
				printf("%.*s  - Serial Number: %s\n", level * 2,
				"                    ", string);
		}
	}

	if (verbose) {
		for (i = 0; i < desc.bNumConfigurations; i++) {
			struct libusb_config_descriptor *config;
			ret = libusb_get_config_descriptor(dev, i, &config);
			if (LIBUSB_SUCCESS != ret) {
				printf("  Couldn't retrieve descriptors\n");
				continue;
			}

			print_configuration(config);

			libusb_free_config_descriptor(config);
		}

		if (handle && desc.bcdUSB >= 0x0201) {
			print_bos(handle);
		}
	}

	if (handle)
		libusb_close(handle);

	return 0;
}

/* Author: Hassan Hakim Khalili
 * Convenience function for finding a device with a particular
 * <tt>serial_number</tt>. The function returns a device handle that 
 * allows you to perform I/O on the device.
 * 
 * Internally, this function calls libusb_open which adds a reference to 
 * the device. This reference is removed during libusb_close().
 *
 * This is a blocking function; requests are sent over the bus to read
 * the serial_number value.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param serial_number the serial_number value to search for
 * \returns a device handle for the found device, or NULL on error
 * or if the device could not be found. */
libusb_device_handle * libusb_open_device_with_serial_number(
	libusb_context *ctx, const char* serial_number )
{
	struct libusb_device **devs;
	struct libusb_device *found = NULL;
	struct libusb_device *dev;
	struct libusb_device_handle *dev_handle = NULL;
	unsigned char string[256];
	size_t i = 0;
	ssize_t cnt = 0; //holding number of devices in list
	int r;

	if ((cnt = libusb_get_device_list(ctx, &devs)) < 0)
	{
		// error occurred
		return NULL;
	}

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
			break;

		r = libusb_open(dev, &dev_handle);
		if (r < 0)
			break;

		// Something is wrong if we can't read serial number index
		// Or is there a case for valid serial number at index 0?
		if (desc.iSerialNumber) {
			r = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, string, sizeof(string));
			if (r > 0)
				if(!strcmp((const char *)string, (const char *)serial_number))
					break; // found
		}

		libusb_close(dev_handle);

	}

	libusb_free_device_list(devs, 1);
	return dev_handle;
}


/* Author: Hassan Hakim Khalili
 * Returns a list of USB devices currently attached to the system, with
 * a <tt>idVendor</tt>/<tt>idProduct</tt> combination.
 * 
 * You are expected to unreference all the devices when you are done with
 * them, and then free the list with libusb_free_device_list(). Note that
 * libusb_free_device_list() can unref all the devices for you. Be careful
 * not to unreference a device you are about to open until after you have
 * opened it.
 *
 * This return value of this function indicates the number of devices in
 * the resultant list. The list is actually one element larger, as it is
 * NULL-terminated.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param list output location for a list of devices. Must be later freed with
 * libusb_free_device_list().
 * \param vendor_id the idVendor value to search for
 * \param product_id the idProduct value to search for
 * \returns the number of devices in the outputted list, or any
 * \ref libusb_error according to errors encountered by the backend.
 */
ssize_t libusb_get_device_list_with_vid_pid(libusb_context *ctx,
	libusb_device ***list, uint16_t vendor_id, uint16_t product_id)
{
	struct libusb_device **devs; // All devices found
	struct libusb_device **found; // List of devices with given vid_pid
	struct libusb_device *dev;
	size_t i = 0;
	size_t N = 0; // Number of devices found
	ssize_t cnt = 0; //holding number of devices in list
	int r; // return value

	if ((cnt = libusb_get_device_list(ctx, &devs)) < 0)
	{
		return -1;
	}

	// Initialise list of found devices to the same size as *devs
	// Last element is NULL to denote the end
	found = (libusb_device**)malloc( (cnt+1) * sizeof(struct libusb_device *) );

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
			goto error;
		if (desc.idVendor == vendor_id && desc.idProduct == product_id) {
			// printf("found\n");
			found[N] = libusb_ref_device(dev);
			++N;
		}
	}

	/* We will almost definitely have less devices found
	 * than originally allocated, so we shall truncate the array.
	 * The last element should be NULL. */
	if(N<cnt){
		struct libusb_device **temp = (libusb_device**)realloc(found, (N+1) * sizeof(struct libusb_device *));
		if (temp != NULL)
			found = temp;
		
		// if (temp == NULL) it's OK, we just return the old array			
	}

	found[N] = NULL;
	*list = found;
	goto out;

error:
	// The libusb_free_device_list looks for a NULL element to terminate. 
	found[N] = NULL;
	libusb_free_device_list(found, 1);
	N = 0;
out:
	libusb_free_device_list(devs, 1);
	return N;
}

#endif

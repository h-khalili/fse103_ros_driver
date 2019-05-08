#include <iostream>
#include <unistd.h>
#include <vector>
#include <bitset>
#include <memory>
#include "libusb-1.1.h"
#include "Iir.h"

#ifndef VARIENSE_FSE103_H
#define VARIENSE_FSE103_H

namespace variense
{

uint16_t vendor_id = 0x16d0;
uint16_t product_id = 0x0c21;

/**
 * An FSE103 data message class. This class represents a data packet
 * sent by the Variense FSE103 force sensor and performs data validation.
 */

struct Fse103_message
{
    uint8_t start_byte;
    uint8_t message_size;
    uint8_t message_type;
    uint32_t timestamp;
    float force_x;
    float force_y;
    float force_z;
    uint8_t end_byte;

    Fse103_message(const unsigned char * data, const bool validate = false)
    {
        start_byte = data[0]; 
        message_size = data[1];
        message_type = data[2];
        memcpy(&timestamp, data+3, sizeof(uint32_t));
        memcpy(&force_x, data+7, sizeof(float));
        memcpy(&force_y, data+11, sizeof(float));
        memcpy(&force_z, data+15, sizeof(float));
        end_byte = data[19];

        // FSE103 packet is big-endian; Linux x86 is little-endian
        _convert_endianness_4B((char *) &timestamp);
        _convert_endianness_4B((char *) &force_x);
        _convert_endianness_4B((char *) &force_y);
        _convert_endianness_4B((char *) &force_z);

        if(validate)
            _validate();
    }

    operator std::string(){
        std::string temp;
        unsigned char data [20];

        data[0] = start_byte; 
        data[1] = message_size;
        data[2] = message_type;
        memcpy(data+3, &timestamp, sizeof(uint32_t));
        memcpy(data+7, &force_x, sizeof(float));
        memcpy(data+11, &force_y, sizeof(float));
        memcpy(data+15, &force_z, sizeof(float));
        data[19] = end_byte;

        // FSE103 packet is big-endian; Linux x86 is little-endian
        _convert_endianness_4B((char *) data+3);
        _convert_endianness_4B((char *) data+7);
        _convert_endianness_4B((char *) data+11);
        _convert_endianness_4B((char *) data+15);

        char hex [6];
        hex[5] = 0;
        for (int i=0; i<20; ++i)
        {
            sprintf(hex, "0x%02x ", data[i]);
            temp += hex;
        }

        // printf("start: 0x%02x, ", start_byte);
        // printf("size: %d, ", message_size);
        // printf("type: 0x%02x ", message_type);
        // printf("timestamp: %d(0x%08x), ", timestamp, timestamp);
        // printf("end: 0x%02x \n", end_byte);
        return temp;
    } 

    private:

    void _validate()
    {
        if(start_byte != 0x0D)
            throw std::runtime_error("Incorrect 'Start of Message' byte (!= 0x0D).");
        if(message_size != 20)
            throw std::runtime_error("Incorrect message size (!= 20).");
        if(message_type != 0x66 && message_type != 0x72)
            throw std::runtime_error("Incorrect data format (=! 0x66 OR != 0x72).");
        if(end_byte != 0xFF)
            throw std::runtime_error("Incorrect 'End of Message' byte (=!0xFF).");  
    }

    // Reverse the endianness of a 4-byte variable
    void _convert_endianness_4B (char * bytes)
    {
        std::swap(bytes[0], bytes[3]);
        std::swap(bytes[1], bytes[2]);
    }

};


constexpr int filter_order = 2; // 2nd order (=1 biquad)

/**
 * A filter class for the FSE103. This class makes use of the C++ 
 * library developed by Bernd Porr to implement a second-order 
 * low-pass Butterworth filter for the force sensor measurements 
 * along its three axes.
 */
class Fse103_filter
{  
    // Cut-off frequency in half-cycles/sample (Wn=fc/(fs/2))
    const float _Wn; 
    Iir::Butterworth::LowPass<filter_order> _filter [3];

    public:

    Fse103_filter(const float Wn): _Wn(Wn)
    {
        for(int i=0; i<3; ++i) _filter[i].setup(2, Wn);
        /*
        std::cout << "Number of stages:" << _filter[0].getNumStages() << std::endl;
        std::cout << "A0:" << _filter[0][0].getA0() << std::endl;
        std::cout << "A1:" << _filter[0][0].getA1() << std::endl;
        std::cout << "A2:" << _filter[0][0].getA2() << std::endl;
        std::cout << "B0:" << _filter[0][0].getB0() << std::endl;
        std::cout << "B1:" << _filter[0][0].getB1() << std::endl;
        std::cout << "B2:" << _filter[0][0].getB2() << std::endl;
        */ 
    }

    // This function performs sample-by-sample filtering (of a vector of length 3)
    const std::vector<float> operator () (const std::vector<float> raw) 
    {
        std::vector<float> temp(3);
        for (int i=0; i<3; ++i) temp[i] = _filter[i].filter(raw[i]);
        return temp;
    }

};

/**
 * The FSE103 force sensor class. This class uses the libusb library
 * to communicate with a Variense FSE103 sensor, read force measurements
 * and optinally apply a low-pass filter.
 */
class Fse103
{
    libusb_context *_ctx;
    libusb_device *_dev;
    libusb_device_handle *_dev_handle;
    std::string _serial_number;
    
    Fse103(){}
    Fse103(const Fse103& obj){}
    Fse103& operator=(const Fse103& obj){}

    // Pointer to a filter for raw force sensor values 
    std::unique_ptr<Fse103_filter> _fse103_filter_ptr;

    public:
    
    const static char FSE103_OUT_ENDPOINT = 0x03;
    const static char FSE103_IN_ENDPOINT = 0x82;
    const static int CONTROL_INTERFACE = 0;
    const static int DATA_INTERFACE = 1;
    const static int FSE103_PACKET_SIZE = 64;
    enum data_format: uint8_t { RAW, CALCULATED };
        
    
    Fse103(std::string SN = "", float Wn = 0):
        _serial_number(SN),
        _ctx(nullptr),
        _dev(nullptr),
        _dev_handle(nullptr),
        _fse103_filter_ptr(nullptr)
    {
        if(Wn>0 && Wn<=1) _fse103_filter_ptr.reset(new Fse103_filter(Wn));
    }

    std::string get_serial_number() { return _serial_number; }
    
    void open(const char reset = 1)    
    {
        ssize_t r; // for return values

        r = libusb_init(&_ctx); 
        if(r < 0)
            throw std::runtime_error(std::string("libusb could not be initialised. ") + libusb_error_name(r));

        if(_serial_number == "")
        {
            _dev_handle = libusb_open_device_with_vid_pid(_ctx, vendor_id, product_id);
            if(_dev_handle)
            {
                unsigned char serial_buffer[256];
                // Returns number of bytes
                // If we already have a handle, reading the serial number should be fine
                r = libusb_get_device_serial_number(_dev_handle, serial_buffer, sizeof(serial_buffer));
                if(r > 0) 
                    _serial_number = (char *) serial_buffer;
            }
        }
        else
            _dev_handle = libusb_open_device_with_serial_number(_ctx, _serial_number.c_str());
    
        if (!_dev_handle)
            throw std::runtime_error(std::string("Variense FSE 103 sensor (") 
                + _serial_number + ") could not be accessed." +  
                " Ensure you have USB access permission and " + 
                "you have entered the correct serial number.");

        // Kernel attaches when USB is plugged in.
        // Kernel detaching works with any available interface number (0 or 1).
        // Use 0 to stay consistent with attach method.
        r = libusb_kernel_driver_active(_dev_handle, CONTROL_INTERFACE);
        if(r < 0)
            throw std::runtime_error(std::string("Cannot interrogate kernel attachment: ") + libusb_error_name(r));
        else
        {
            // If a kernel driver is active on the interface, detach it.
            if(r)
            {
                r = libusb_detach_kernel_driver(_dev_handle, CONTROL_INTERFACE);
                if(r < 0)
                    throw std::runtime_error(std::string("Cannot detach kernel driver: ") + libusb_error_name(r));
            }
        }
         
        // If we claim interface 0 instead of 1, and transmit/receive data
        // over the bus, we wouldn't be able to attach the kernel again.
    	r = libusb_claim_interface(_dev_handle, DATA_INTERFACE); 
        if(r < 0)
        {
            // To avoid errors when closing
            libusb_close(_dev_handle);
            _dev_handle = NULL;

            throw std::runtime_error(std::string("Cannot claim interface: ") +
            libusb_error_name(r) + ". Is the sensor driver ("
                 + _serial_number + ") already running?");
        }

        if(reset)
        {
            r = libusb_reset_device(_dev_handle);
            if (r < 0) {
                std::cerr << "Error resetting the device: " << libusb_error_name(r) << std::endl; 
            }
            std::cerr << "Resetting the force sensor ..." << std::endl;;
        }

    }

    ~Fse103()
    {
        // if(_fse103_filter_ptr) delete _fse103_filter_ptr; // Replaced by unique_ptr
        close();
    }

    void close()
    {
        ssize_t r;
        if(_dev_handle)
        {
            // This must match claimed interface
            r = libusb_release_interface(_dev_handle, DATA_INTERFACE); 
            if(r < 0)
                std::cerr<<"Could not release USB interface: "<< libusb_error_name(r) << std::endl;

            r = libusb_kernel_driver_active(_dev_handle, CONTROL_INTERFACE);
            if(r < 0)
                std::cerr << "Cannot interrogate kernel attachment: " << libusb_error_name(r) << std::endl;
            else
            {
                // Attach kernel driver, only if detached.
                if(!r)
                {
                    // Only works with interface 0
                    r = libusb_attach_kernel_driver(_dev_handle, CONTROL_INTERFACE);
                    if(r < 0)
                        std::cerr<<"Could not attach kernel driver: "<< libusb_error_name(r) << std::endl;
                }
            }

            libusb_close(_dev_handle);
            _dev_handle = NULL;
        }
        if(_ctx) libusb_exit(_ctx);
    }

    void initialise()
    {
        ssize_t r;
        int bytes_written;

        r = libusb_bulk_transfer(_dev_handle, (FSE103_OUT_ENDPOINT | LIBUSB_ENDPOINT_OUT), 
            (unsigned char*)"z", 1, &bytes_written, 1000); 
        if(r < 0)
            throw std::runtime_error(std::string("Could not initialise sensor: ") + libusb_error_name(r));

        // Remove transient packets
        for (int i=0; i<5; ++i)
            read_message();
    }

    int set_data_format(data_format d)
    {
        ssize_t r;
        unsigned char mode [1]; // Byte array to write
        mode[0] = (d == RAW) ? 'r' : 'f'; 
        int bytes_written;

        std::cout<<"Setting data format to "<< (mode[0] == 'r' ? "Raw ..." : "Calculated ...") << std::endl;
        r = libusb_bulk_transfer(_dev_handle, (FSE103_OUT_ENDPOINT | LIBUSB_ENDPOINT_OUT), 
            mode, 1, &bytes_written, 1000); 
        if(r < 0) 
            throw std::runtime_error(std::string("Could not set sensor data format: ") + libusb_error_name(r));

        // Remove transient packets
        for (int i=0; i<5; ++i)
            read_message();
    }

    const Fse103_message read_message(bool validate = false)
    {
        ssize_t r;
        unsigned char data [FSE103_PACKET_SIZE];
	    int bytes_read; // Used to find out how many bytes were written
        r = libusb_bulk_transfer(_dev_handle, (FSE103_IN_ENDPOINT | LIBUSB_ENDPOINT_IN), data, FSE103_PACKET_SIZE, &bytes_read, 1000); 
        if(r < 0) 
            throw std::runtime_error(std::string("Could not read sensor data: ") + libusb_error_name(r));

        // for (int i=0; i<20; ++i)
        //     printf("0x%02x ", data[i]);

        Fse103_message msg(data, validate);
        // std::cout << std::string(msg) << std::endl;

        return msg;
    }

    // Returns a vector of three elements
    const std::vector<float> read_force()
    {
        bool validate = true;  // Validate read message
        const Fse103_message msg = read_message(validate);

        std::vector<float> force = {msg.force_x, msg.force_y, msg.force_z};

        if(_fse103_filter_ptr)
            force = (*_fse103_filter_ptr)(force);

        return force;
    }

};


} // namespace variense

#endif // #define VARIENSE_FSE103_H
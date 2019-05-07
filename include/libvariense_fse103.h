#include <iostream>
#include <unistd.h>
#include <vector>
#include <boost/endian/conversion.hpp>
#include <bitset>
#include "libusb-1.1.h"
#include "Iir.h"

namespace variense
{

uint16_t vendor_id = 0x16d0;
uint16_t product_id = 0x0c21;

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

    Fse103_message(const unsigned char * data)
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
        convert_endianness_4B((char *) &timestamp);
        convert_endianness_4B((char *) &force_x);
        convert_endianness_4B((char *) &force_y);
        convert_endianness_4B((char *) &force_z);
    }

    private:
    // Reverse the endianness of a 4-byte variable
    void convert_endianness_4B (char * bytes)
    {
        std::swap(bytes[0], bytes[3]);
        std::swap(bytes[1], bytes[2]);
    }

};


constexpr int filter_order = 2; // 2nd order (=1 biquad)

class Fse103_filter
{  
    // Cut-off frequency in half-cycles/sample (Wn=fc/(fs/2))
    const float _Wn; 
    Iir::Butterworth::LowPass<filter_order> _filter [3];

    public:

    Fse103_filter(const float Wn): _Wn(Wn)
    {
        for(int i=0; i<3; ++i) _filter[i].setup(2, Wn);
        std::cout << "Number of stages:" << _filter[0].getNumStages() << std::endl;
        std::cout << "A0:" << _filter[0][0].getA0() << std::endl;
        std::cout << "A1:" << _filter[0][0].getA1() << std::endl;
        std::cout << "A2:" << _filter[0][0].getA2() << std::endl;
        std::cout << "B0:" << _filter[0][0].getB0() << std::endl;
        std::cout << "B1:" << _filter[0][0].getB1() << std::endl;
        std::cout << "B2:" << _filter[0][0].getB2() << std::endl;
    }

    // This function performs sample-by-sample filtering (of a vector of length 3)
    const std::vector<float> operator () (const std::vector<float> raw) 
    {
        std::vector<float> temp(3);
        for (int i=0; i<3; ++i) temp[i] = _filter[i].filter(raw[i]);
        return temp;
    }

};

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
    Fse103_filter * _fse103_filter_ptr;

    public:
    
    const static char FSE103_OUT_ENDPOINT = 0x3;
    const static char FSE103_IN_ENDPOINT = 0x82;
    const static int DATA_INTERFACE = 1;
    const static int FSE103_PACKET_SIZE = 64;
    enum data_type: uint8_t { RAW, CALCULATED };
        
    
    Fse103(std::string SN, float Wn = 0):
        _serial_number(SN),
        _ctx(nullptr),
        _dev(nullptr),
        _dev_handle(nullptr),
        _fse103_filter_ptr(nullptr)
    {
        if(Wn>0 && Wn<=1) _fse103_filter_ptr = new Fse103_filter(Wn);
        std::cout<<"Constructor "<<std::endl;
    }
    
    void open(const char reset = 1)    
    {
        ssize_t r; // for return values

        r = libusb_init(&_ctx); 
        if(r < 0) {
            std::cout<<"Init Error "<<r<<std::endl; //there was an error
            throw std::runtime_error("libusb could not be initialised.");
        }
        std::cout<<"libusb initialised "<<r<<std::endl;

        if(_serial_number == "")
        {
            std::cout<<"No serial number given"<<r<<std::endl;
            _dev_handle = libusb_open_device_with_vid_pid(_ctx, vendor_id, product_id);
            if(_dev_handle)
            {
                unsigned char serial_buffer[256];
                r = libusb_get_device_serial_number(_dev_handle, serial_buffer, sizeof(serial_buffer));
                if(r > 0)
                {
                    _serial_number = (char *) serial_buffer;
                    std::cout<<"Serial number obtained successfully: "<<_serial_number<<std::endl;
                }
            }
        }
        else
        {
            std::cout<<"Serial number given:" << _serial_number <<std::endl;
            _dev_handle = libusb_open_device_with_serial_number(_ctx, _serial_number.c_str());
        }
    
        if (_dev_handle)
        {
            libusb_print_device(libusb_get_device(_dev_handle), 0, 0);
            if(_dev_handle)
            {
                std::cout<<"dev found?: "<<std::endl;
                unsigned char serial_buffer[256];
                r = libusb_get_device_serial_number(_dev_handle, serial_buffer, sizeof(serial_buffer));
                if(r > 0)
                {
                    _serial_number = (char *) serial_buffer;
                    std::cout<<"Serial number obtained successfully: "<<_serial_number<<std::endl;
                }
            }           
        }
        else
        {
            std::string err = "Variense FSE 103 sensor (" + _serial_number + ") could not be opened. Do you have USB access permission?";
            // throw std::runtime_error("Variense FSE 103 sensor (%s) could not be opened. Do you have USB access permission?");
            throw std::runtime_error(err);
        }

        // kernel attaches when USB is plugged in
        // This works with any available interface number (0 or 1).
        // Use 0 to stay consistent with attach method.
        if(libusb_kernel_driver_active(_dev_handle, 1) == 1) { 
            std::cout<<"Kernel Driver Active"<<std::endl;
            if(libusb_detach_kernel_driver(_dev_handle, 1) == 0)
                std::cout<<"Kernel Driver Detached!"<<std::endl;
        }

        // If we claim interface 0 instead of 1, and transmit/receive data
        // over the bus, we wouldn't be able to attach the kernel again.
    	r = libusb_claim_interface(_dev_handle, DATA_INTERFACE); 
        if(r < 0) {
            std::cout<<"Cannot Claim Interface"<<std::endl;
            std::string err = "Cannot claim interface. Is the sensor driver (" + _serial_number + ") already running?";
            throw std::runtime_error(err);
        }
        std::cout<<"Claimed Interface"<<std::endl;

        if(reset)
        {
            r = libusb_reset_device(_dev_handle);
            if (r < 0) {
                std::cerr << "Error resetting the device: " << libusb_error_name(r) << std::endl; 
            }
            std::cerr << "Resetting the device: " << std::endl;;
        }

    }

    ~Fse103()
    {
        std::cout << "Destructor: " << _serial_number << std::endl;
        if(_fse103_filter_ptr) delete _fse103_filter_ptr;
        close();
    }

    void close()
    {
        ssize_t ret;
        if(_dev_handle)
        {
            ret = libusb_release_interface(_dev_handle, DATA_INTERFACE); 
            if(ret !=0) {
                std::cout<<"Cannot Release Interface 1"<<std::endl;
                // return;
            }
            std::cout<<"Released Interface 1"<<std::endl;

            ret = libusb_attach_kernel_driver(_dev_handle, 0);
            // puts(libusb_strerror((libusb_error)r));
            switch (ret)
            {
                case 0:
                    puts("success0");
                    break;

                case LIBUSB_ERROR_NOT_FOUND:
                    puts("No kernel driver was active.0");
                    break;

                case LIBUSB_ERROR_INVALID_PARAM:
                    puts("Error: Interface does not exist.0");
                    break;

                case LIBUSB_ERROR_NO_DEVICE:
                    puts("LIBUSB_ERROR_NO_DEVICE0");
                    break;

                case LIBUSB_ERROR_NOT_SUPPORTED:
                    puts("LIBUSB_ERROR_NOT_SUPPORTED0");
                    break;            

                default:
                    puts("unknown error0");
                    break;
            }
            libusb_close(_dev_handle);
        }
        if(_ctx) libusb_exit(_ctx);
    }


    int initialise()
    {
        ssize_t r;
        int bytes_written;
        std::cout<<"initializing"<<std::endl;
        r = libusb_bulk_transfer(_dev_handle, (FSE103_OUT_ENDPOINT | LIBUSB_ENDPOINT_OUT), 
            (unsigned char*)"z", 1, &bytes_written, 1000); 
        if(r == 0 ) //we wrote the 4 bytes successfully
            std::cout<<"Writing Successful!" << bytes_written <<std::endl;
        else
            std::cout<<"Write Error"<<std::endl;

        // Remove transient packets
        for (int i=0; i<5; ++i)
            read_message();

        return r;
    }

    int set_data_type(data_type d)
    {
        ssize_t r;
        unsigned char mode [1];
        mode[0] = (d == RAW) ? 'r' : 'f'; 
        int bytes_written;
        std::cout<<"Setting data type to "<< mode[0] << std::endl;
        r = libusb_bulk_transfer(_dev_handle, (FSE103_OUT_ENDPOINT | LIBUSB_ENDPOINT_OUT), 
            mode, 1, &bytes_written, 1000); //my device's out endpoint was 2, found with trial- the device had 2 endpoints: 2 and 129
        if(r == 0 ) //we wrote the 4 bytes successfully
            std::cout<<"Writing Successful!" << bytes_written <<std::endl;
        else
            std::cout<<"Write Error"<<std::endl;

        // Remove transient packets
        for (int i=0; i<5; ++i)
            read_message();
    }

    const Fse103_message read_message()
    {
        ssize_t r; //for return values
        unsigned char data [FSE103_PACKET_SIZE];
	    int bytes_read; //used to find out how many bytes were written
        // std::cout<<"readinf Data..."<<std::endl;
        r = libusb_bulk_transfer(_dev_handle, (0x82 | LIBUSB_ENDPOINT_IN), data, FSE103_PACKET_SIZE, &bytes_read, 1000); 
        // if(r == 0 ) //we wrote the 4 bytes successfully
        //     std::cout<<"reading Successful!" << bytes_read <<std::endl;
        // else
        //     std::cout<<"read Error"<<std::endl;
            // TODO what happens if we read more btyes


        Fse103_message msg(data);


        // puts("bytes:");
        // for (int i =0; i<20; ++i)
        // {
        //     printf("0x%02x ", data[i]);    
        // }
        // puts("");



        return msg;
    }

    // Returns a vector of three elements
    const std::vector<float> read_force()
    {
        const Fse103_message msg = read_message();
        // printf("start: 0x%02x, ", msg.start_byte);
        // printf("size: %d, ", msg.message_size);
        // printf("type: 0x%02x ", msg.message_type);
        // printf("timestamp: %d(0x%08x), ", msg.timestamp, msg.timestamp);
        // printf("end: 0x%02x \n", msg.end_byte);
        std::vector<float> force = {msg.force_x, msg.force_y, msg.force_z};

        if(_fse103_filter_ptr)
            force = (*_fse103_filter_ptr)(force);

        return force;
    }

};


} // end namespace

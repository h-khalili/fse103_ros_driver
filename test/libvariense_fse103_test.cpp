#include "../include/variense_fse103/libvariense_fse103.h"
#include "../include/variense_fse103/iir/Cascade.cpp"
#include "../include/variense_fse103/iir/Butterworth.cpp"
#include "../include/variense_fse103/iir/Biquad.cpp"
#include "../include/variense_fse103/iir/PoleFilter.cpp"


int main()
{

    std::string SN = "103EAA8876";
    SN = "";
    variense::Fse103 force_sensor(SN);
    try
    {
        std::cout<<"Try opening force sensor "<<std::endl;
        force_sensor.open();
    }
    catch(const std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    // force_sensor.initialise();
    force_sensor.set_data_format(variense::Fse103::RAW);

    for(int i=0; i<5; ++i)
    {
        const std::vector<float> force = force_sensor.read_force();
        std::cout << force[0] << ", " << force[1] << ", " << force[2] << std::endl; 
    }

    force_sensor.set_data_format(variense::Fse103::CALCULATED);

    for(int i=0; i<5; ++i)
    {
        const std::vector<float> force = force_sensor.read_force();
        std::cout << force[0] << ", " << force[1] << ", " << force[2] << std::endl; 
    }
    return 0;
}

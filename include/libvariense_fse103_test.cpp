#include "libvariense_fse103.h"


int main()
{
    std::string SN = "103EAA8876";
    variense::Fse103 force_sensor(SN);
    try
    {
        force_sensor.open();
    }
    catch(const std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    // force_sensor.initialise();
    force_sensor.set_data_type(variense::Fse103::data_type::RAW);

    for(int i=0; i<5; ++i)
    {
        const std::vector<float> force = force_sensor.read_force();
        std::cout << force[0] << ", " << force[1] << ", " << force[2] << std::endl; 
    }

    force_sensor.set_data_type(variense::Fse103::CALCULATED);

    for(int i=0; i<5; ++i)
    {
        const std::vector<float> force = force_sensor.read_force();
        std::cout << force[0] << ", " << force[1] << ", " << force[2] << std::endl; 
    }
    return 0;
}

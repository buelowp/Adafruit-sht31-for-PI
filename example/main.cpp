#include <sht31d.h>
#include <iostream>

int main(int argc, char *argv[]) 
{
    SHT31D *sht31d = new SHT31D(4);
    float temp = 0;
    float humidity = 0;

    if (sht31d->isOpen()) {
        uint32_t serial = 0;
        SHT31D::sht31dreturn rval = sht31d->serialNumber(serial);
        if (rval == SHT31D::sht31dreturn::SHT31_OK) {
            std::cout << "Found device, serial number " << serial << std::endl;
        }
        else {
            std::cout << "Problems, rval = " << rval << std::endl;
        }

        if ((rval = sht31d->values(temp, humidity)) != SHT31D::sht31dreturn::SHT31_OK) {
            std::cout << "Unable to get temp/humidity : " << sht31d->lastError() << std::endl;
        }
        else {
            std::cout << "Temp: " << temp * 1.8 + 32 << ", Humidity: " << humidity << std::endl;
        }
    }
    else {
        std::cout << "Device is not open" << std::endl;
    }

    delete sht31d;
    return 0;
}

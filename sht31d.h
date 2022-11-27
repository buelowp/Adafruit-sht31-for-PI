#pragma once

#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <cstdio>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <string>

#define SHT31_INTERFACE_ADDR       1
#define SHT31_DEFAULT_ADDR         0x44
#define SHT31_READ_SERIALNO        0x3780
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06 // Doesn't work on PI
#define SHT31_MEAS_MEDREP_STRETCH  0x2C0D // Seems to work on PI but shouldn't
#define SHT31_MEAS_LOWREP_STRETCH  0x2C10 // Seems to work on PI but shouldn't
#define SHT31_MEAS_HIGHREP         0x2400 // Doesn't work on PI
#define SHT31_MEAS_MEDREP          0x240B
#define SHT31_MEAS_LOWREP          0x2416
#define SHT31_READSTATUS           0xF32D
#define SHT31_CLEARSTATUS          0x3041
#define SHT31_SOFTRESET            0x30A2
#define SHT31_HEATER_ENABLE        0x306D
#define SHT31_HEATER_DISABLE       0x3066

#define SHT32_DEFAULT_READ         SHT31_MEAS_MEDREP

// Tempoary measure to fix the first write fail
#define SHT31D_FIX_INITIAL_FAIL 1

class SHT31D
{
public:
    SHT31D(uint8_t device = 0, uint8_t address = 0x44);
    ~SHT31D();

    enum sht31dreturn:ssize_t
    {
        SHT31_OK = 0,
        SHT31_CRC_CHECK_FAILED = 1,
        SHT31_BAD = 2,
        SHT31_READ_FAILED = 3,
        SHT31_WRITE_FAILED = 4,
        SHT31_NOT_OPEN = 5
    };

    sht31dreturn values(float &temp, float &humidity);
    sht31dreturn status(uint16_t *rtnbuf);
    sht31dreturn serialNumber(uint32_t &serial);
    sht31dreturn clearStatus();
    sht31dreturn reset();
    sht31dreturn heater(bool state);
    bool is_open();

private:
    void delay (unsigned int delay);
    uint8_t crc8(const uint8_t *data, int len);
    sht31dreturn query(uint16_t sndword, uint8_t *buffer, int readsize);

    int m_fd;
    std::string m_device;
    std::string m_lastError;
    int m_lastErrno;
    bool m_open;
    uint8_t m_address;
};


#include "sht31d.h"

SHT31D::SHT31D(uint8_t device, uint8_t address)
{
    m_device = "/dev/i2c-";
    m_device += std::to_string(address);
    if ((m_fd = open(m_device.c_str(), O_RDWR)) < 0) {
        m_open = false;
        m_lastErrno = errno;
        m_lastError = strerror(errno);
        return;
    }

    if (ioctl(m_fd, I2C_SLAVE, address) < 0) {
        m_lastErrno = errno;
        m_lastError = strerror(errno);
        close(m_fd);
        m_open = false;
        return;
    }
  // Do an itial read & write of basically nothing.
  // Still working on this, but without it the first command issued failes, these will also usually fail
#ifdef SHT31D_FIX_INITIAL_FAIL
    uint8_t buf = 0x00;
    write(m_fd, &buf, 1);
    read(m_fd, &buf, 1);
#endif
    m_open = true;
}

SHT31D::~SHT31D()
{
    close(m_fd);
    m_open = false;
}

bool SHT31D::is_open()
{
    return m_open;
}

void SHT31D::delay(unsigned int delay)
{
    struct timespec sleeper;

    sleeper.tv_sec  = (time_t)(delay / 1000) ;
    sleeper.tv_nsec = (long)(delay % 1000) * 1000000 ;

    nanosleep (&sleeper, NULL) ;
}

uint8_t SHT31D::crc8(const uint8_t* data, int len)
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;
    int j;
    int i;

    for (j = len; j; --j ) {
        crc ^= *data++;

        for ( i = 8; i; --i ) {
            crc = ( crc & 0x80 ) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

SHT31D::sht31dreturn SHT31D::query(uint16_t sndword, uint8_t* buffer, int readsize)
{
    SHT31D::sht31dreturn rtn;
    int sendsize = 2;
    uint8_t snd[sendsize];

    m_lastErrno = 0;
    m_lastError.clear();

    if (!m_open)
      return SHT31_NOT_OPEN;

    // big-endian - Split the 16bit word into two 8 bits that are flipped.
    snd[0]=(sndword >> 8) & 0xff;
    snd[1]=sndword & 0xff;

    if (write(m_fd, snd, sendsize) != sendsize) {
        m_lastErrno = errno;
        m_lastError = strerror(errno);
        return SHT31_WRITE_FAILED;
    }

    if (readsize > 0) {
        delay(10);
        if (read(m_fd, buffer, readsize) < readsize) {
            m_lastErrno = errno;
            m_lastError = strerror(errno);
            return SHT31_WRITE_FAILED;
        }
    }

    return SHT31_OK;
}

SHT31D::sht31dreturn SHT31D::serialNumber(uint32_t &serial)
{
    uint8_t buf[10];
    sht31dreturn rtn;

    m_lastErrno = 0;
    m_lastError.clear();

    if (!m_open)
        return SHT31_NOT_OPEN;

    if ((rtn = query(SHT31_READ_SERIALNO, buf, 6)) != SHT31_OK) {
        return rtn;
    }
    else {
        serial = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[3] << 8) | (uint32_t)buf[4];

        if (buf[2] != crc8(buf, 2) || buf[5] != crc8(buf+3, 2))
            return SHT31_CRC_CHECK_FAILED;
    }

    return SHT31_OK;
}

SHT31D::sht31dreturn SHT31D::values(float &temp, float &hum)
{
    uint8_t buf[10];
    sht31dreturn rtn;

    m_lastErrno = 0;
    m_lastError.clear();

    if ((rtn = query(SHT32_DEFAULT_READ, buf, 6)) != SHT31_OK) {
        return rtn;
    }
    else {
        if (buf[2] != crc8(buf, 2) || buf[5] != crc8(buf+3, 2))
            return SHT31_CRC_CHECK_FAILED;

        uint16_t ST, SRH;
        ST = buf[0];
        ST <<= 8;
        ST |= buf[1];

        SRH = buf[3];
        SRH <<= 8;
        SRH |= buf[4];

        temp = -45.0 + (175.0 * ((float) ST / (float) 0xFFFF));
        hum = 100.0 * ((float) SRH / (float) 0xFFFF);

    }

    return SHT31_OK;
}


SHT31D::sht31dreturn SHT31D::status(uint16_t *rtnbuf)
{
    uint8_t buf[10];
    SHT31D::sht31dreturn rtn;

    if ((rtn = query(SHT31_READSTATUS, buf, 3)) != SHT31_OK) {
        return rtn;
    }
    else {
        *rtnbuf = buf[0];
        *rtnbuf <<= 8;
        *rtnbuf |= buf[1];

        if (buf[2] != crc8(buf, 2))
            return SHT31_CRC_CHECK_FAILED;
    }

    return SHT31_OK;
}

SHT31D::sht31dreturn SHT31D::clearStatus()
{
    return query(SHT31_CLEARSTATUS, NULL, 0);
}

SHT31D::sht31dreturn SHT31D::heater(bool state)
{
    if (state) {
        return query(SHT31_HEATER_ENABLE, NULL, 0);
    }
    return query(SHT31_HEATER_DISABLE, NULL, 0);
}

SHT31D::sht31dreturn SHT31D::reset()
{
    return query(SHT31_SOFTRESET, NULL, 0);
}

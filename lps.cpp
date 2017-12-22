#include "lps.h"
#include <stdexcept>

void lps::handle::open(const comm_config & config)
{
  if (!config.use_sensor)
  {
    throw std::runtime_error("LPS configuration is null.");
  }

  this->config = config;
  i2c.open(config.i2c_bus_name);
}

void lps::handle::write_reg(reg_addr addr, uint8_t value)
{
  i2c.write_two_bytes(config.i2c_address, addr, value);
}

void lps::handle::enable()
{
  if (config.device == LPS25H)
  {
    // 0xB0 = 0b10110000
    // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
    write_reg(CTRL_REG1, 0b10110000);
  }
  else
  {
    std::runtime_error("Cannot enable unknown LPS device.");
  }
}

void lps::handle::read_altimeter()
{
  {
    // pressure
    uint8_t block[3];
    // assert MSB to enable register address auto-increment
    i2c.write_byte_and_read(config.i2c_address, 0x80 | PRESS_OUT_XL, block, sizeof(block));
    // combine bytes
    p = (int32_t)(int8_t)block[2] << 16 | (uint16_t)block[1] << 8 | block[0];
  }

  {
    // temperature
    uint8_t block[2];
    // assert MSB to enable register address auto-increment
    i2c.write_byte_and_read(config.i2c_address, 0x80 | TEMP_OUT_L, block, sizeof(block));
    // combine bytes
    t = (int16_t)(block[1] << 8 | block[0]);
  }
  
}

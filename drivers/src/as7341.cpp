#include "../include/as7341.hpp"
#include <libhal-util/i2c.hpp>
#include <libhal/timeout.hpp>

namespace sjsu::drivers {
as7341::as7341(hal::v5::strong_ptr<hal::i2c> p_i2c,
               hal::v5::strong_ptr<hal::steady_clock> p_clock,
               hal::v5::strong_ptr<hal::serial> p_terminal)
  : m_i2c(p_i2c)
  , m_clock(p_clock)
  , m_terminal(p_terminal)
{
  default_enable();
}

void as7341::default_enable()
{
  // set PON = 1, then config device and enable interrupts before set SP_EN = 1

  // start sensor (PON = 1)
  hal::byte int_pon = 0x01;
  std::array<hal::byte, 2> write_buff = { register_set::enable, int_pon };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // config sensor
  hal::byte int_atime = 0x64;  // 100
  write_buff = { register_set::atime, int_atime };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  hal::byte int_asteph = 0x03;  // 999 (2.78 ms)
  write_buff = { register_set::asteph, int_asteph };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  hal::byte int_astepl = 0xE7;  // 999 (2.78 ms)
  write_buff = { register_set::astepl, int_astepl };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  hal::byte int_gain = 0x05;  // 16x (0 to 65535)
  write_buff = { register_set::gain, int_gain };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // config smux
  smux_config();

  hal::byte int_smux =
    0x10;  // 2 (write SMUX configuration from RAM to SMUX chain)
  write_buff = { register_set::smux, int_smux };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // start operation
  hal::byte int_smuxen = 0x11;  // PON = 1, SMUXEN = 1
  write_buff = { register_set::enable, int_smuxen };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  hal::byte int_spen = 0x03;  // PON = 1, SP_EN = 1
  write_buff = { register_set::enable, int_spen };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());
}

void as7341::smux_config()
{  // adafruit smux table (F1 – F4 + Clear + NIR)
  std::array<hal::byte, 2> write_buff;

  write_buff = { 0x00, 0x30 };  // F3 left set to ADC2
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x01, 0x01 };  // F1 left set to ADC0
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x02, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x03, 0x00 };  // F8 left disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x04, 0x00 };  // F6 left disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x05,
                 0x42 };  // F4 left connected to ADC3/f2 left connected to ADC1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x06, 0x00 };  // F5 left disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x07, 0x00 };  // F7 left disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x08, 0x50 };  // CLEAR connected to ADC4
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x09, 0x00 };  // F5 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0A, 0x00 };  // F7 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0B, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0C, 0x20 };  // F2 right connected to ADC1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0D, 0x04 };  // F4 right connected to ADC3
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0E, 0x00 };  // F6/F8 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0F, 0x30 };  // F3 right connected to AD2
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x10, 0x01 };  // F1 right connected to AD0
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x11, 0x50 };  // CLEAR right connected to AD4
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x12, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x13, 0x06 };  // NIR connected to ADC5
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());
}

hal::u16 as7341::readChannel(adc_channel channel)
{
  // return channel value
  hal::byte channel_register = static_cast<hal::byte>(channel);
  std::array<hal::byte, 2> data{};

  std::array<hal::byte, 1> write_to_address = { channel_register };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data);

  return static_cast<hal::u16>(data[0]) | (static_cast<hal::u16>(data[1]) << 8);
}

std::array<hal::u16, 6> as7341::readAllChannels()
{
  // read CH0 - CH5 (6 values)
  // return a struct/array with all channel values

  std::array<hal::byte, 16> buff{};
  std::array<hal::byte, 1> write_to_address = { static_cast<hal::byte>(
    adc_channel::ch0data) };
  hal::write_then_read(*m_i2c, m_address, write_to_address, buff);

  for (int i = 0; i < 6; i++) {
    int idx = 2 * i;
    channel_read[i] = static_cast<hal::u16>(buff[idx]) |
                      (static_cast<hal::u16>(buff[idx + 1]) << 8);
  }

  return channel_read;
}

}  // namespace sjsu::drivers
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
  std::array<hal::byte, 2> write_buff;

  // start sensor (PON = 1)
  write_buff = { register_set::enable, 0x01 };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // config sensor
  write_buff = { register_set::atime, 0x64 };  // 100
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  write_buff = { register_set::asteph, 0x03 };  // 999 (2.78 ms)
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  write_buff = { register_set::astepl, 0xE7 };  // 999 (2.78 ms)
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  write_buff = { register_set::gain, 0x05 };  // 16x (0 to 65535)
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // (default) config smux
  set_smux(true);
}

void as7341::set_smux(bool f1f4)
{
  std::array<hal::byte, 2> write_buff;

  write_buff = { register_set::enable, 0x01 };  // PON = 1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  // config smux
  if (f1f4) {
    F1F4_Clear_NIR();
  } else {
    F5F8_Clear_NIR();
  }

  write_buff = { register_set::smux, 0x10 };  // SMUX_CMD = 2
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  hal::delay(*m_clock, 10ms);

  write_buff = { register_set::enable, 0x03 };  // PON = 1, SP_EN = 1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());
}

std::array<hal::u16, 6> as7341::readAllChannels()
{
  // read CH0 - CH5 (6 values)
  // return a struct/array with all channel values

  std::array<hal::byte, 12> buff{};
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

void as7341::F1F4_Clear_NIR()
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

void as7341::F5F8_Clear_NIR()
{  // adafruit smux table (F5 – F8 + Clear + NIR)
  std::array<hal::byte, 2> write_buff;

  write_buff = { 0x00, 0x00 };  // F3 left disable
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x01, 0x00 };  // F1 left disable
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x02, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x03, 0x40 };  // F8 left connected to ADC3
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x04, 0x02 };  // F6 left connected to ADC1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x05, 0x00 };  // F4 / F2 disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x06, 0x10 };  // F5 left connected to ADC0
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x07, 0x03 };  // F7 left connected to ADC2
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x08, 0x50 };  // CLEAR connected to ADC4
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x09, 0x10 };  // F5 right connected to ADC0
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0A, 0x03 };  // F7 right connected to ADC2
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0B, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0C, 0x00 };  // F2 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0D, 0x00 };  // F4 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = {
    0x0E, 0x24
  };  // F8 right connected to ADC2 / F6 right connected to ADC1
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x0F, 0x00 };  // F3 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x10, 0x00 };  // F1 right disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x11, 0x50 };  // CLEAR right connected to AD4
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x12, 0x00 };  // reserved or disabled
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  write_buff = { 0x13, 0x06 };  // NIR connected to ADC5
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());
}

}  // namespace sjsu::drivers
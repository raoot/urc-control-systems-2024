#pragma once
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

using namespace std::chrono_literals;

namespace sjsu::drivers {
class as7341
{
private:
  void write_register(hal::byte register_address, hal::byte value);
  void read_register(hal::byte register_address, std::span<hal::byte> out);

  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::serial> m_terminal;
  hal::byte m_address = 0x39;

  std::array<hal::u16, 6> channel_read{};  // read 6 channels

public:
  as7341(hal::v5::strong_ptr<hal::i2c> p_i2c,
         hal::v5::strong_ptr<hal::steady_clock> p_clock,
         hal::v5::strong_ptr<hal::serial> p_terminal);

  /// @brief list of other registers for as7341
  enum register_set
  {
    enable = 0x80,
    atime = 0x81,
    astepl = 0xCA,
    asteph = 0xCB,
    gain = 0xAA,
    smux = 0xAF,
  };

  /// @brief list of channel registers for as7341
  enum adc_channel
  {
    // store low addresses
    ch0data = 0x95,
    ch1data = 0x97,
    ch2data = 0x99,
    ch3data = 0x9B,
    ch4data = 0x9D,
    ch5data = 0x9F,
  };

  void default_enable();
  void F1F4_Clear_NIR();
  void F5F8_Clear_NIR();
  void set_smux(bool f1f4);

  std::array<hal::u16, 6> readAllChannels();
};
}  // namespace sjsu::drivers
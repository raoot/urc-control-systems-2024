#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/as7341.hpp"

namespace sjsu::drivers {
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto counter = resources::clock();
  auto terminal = resources::console();
  hal::print<64>(*terminal, "Clock and Console Created \n");
  auto i2c2 = resources::i2c();
  hal::print<64>(*terminal, "i2c Created \n");
  using namespace std::literals;

  as7341 as7341_sensor = as7341(i2c2, counter, terminal);

  hal::delay(*counter, 1000ms);

  hal::print<64>(*terminal, "Sensor Created \n");
  hal::delay(*counter, 150ms);

  hal::print<64>(*terminal, "Loop \n");
  while (true) {
    hal::delay(*counter, 500ms);

    std::array<hal::u16, 6> channel = as7341_sensor.readAllChannels();

    // 515nm & 555nm
    // hal::print<64>(*terminal, "F1 415nm: %u\n", channel[0]);
    // hal::print<64>(*terminal, "F2 445nm: %u\n", channel[1]);
    // hal::print<64>(*terminal, "F3 480nm: %u\n", channel[2]);
    hal::print<64>(*terminal, "F4 515nm: %u\n", channel[3]);  // cyan
    hal::print<64>(*terminal, "F5 555nm: %u\n", channel[4]);  // green
    // hal::print<64>(*terminal, "F6 590nm: %u\n", channel[5]);
  }
}
}  // namespace sjsu::drivers
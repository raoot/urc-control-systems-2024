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

    as7341_sensor.set_smux(true);
    hal::delay(*counter, 300ms);
    std::array<hal::u16, 6> F1F4 = as7341_sensor.readAllChannels();

    as7341_sensor.set_smux(false);
    hal::delay(*counter, 300ms);
    std::array<hal::u16, 6> F5F8 = as7341_sensor.readAllChannels();

    // CLEAR (ADC Count Range: 0 – 65535)
    hal::print<64>(*terminal, "CLEAR (F1F5): %u\n", F1F4[4]);
    hal::print<64>(*terminal, "CLEAR (F5F8): %u\n", F5F8[4]);

    // 515nm & 555nm (ADC Count Range: 0 – 65535)
    hal::print<64>(*terminal, "F4 515nm: %u\n", F1F4[3]);  // cyan
    hal::print<64>(*terminal, "F5 555nm: %u\n", F5F8[0]);  // green
    // apply a digital filter to conjugate values
  }
}
}  // namespace sjsu::drivers
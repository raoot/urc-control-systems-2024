#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <science/resource_list.hpp>

#include "../hardware_map.hpp"
#include "../include/as7341.hpp"

constexpr int sample_rate = 20;

// digital filter to conjugate values
// scipy.signal.firwin2(20, [0, fs / 2], [1.0, 0], fs = fs)
constexpr std::array<float, sample_rate> filter_const = {
  9.66174445e-05, 1.55986763e-04, 3.33618583e-04, 7.15587299e-04,
  1.46574468e-03, 2.93946352e-03, 6.05407192e-03, 1.38743094e-02,
  4.26119415e-02, 4.02822938e-01, 4.02822938e-01, 4.26119415e-02,
  1.38743094e-02, 6.05407192e-03, 2.93946352e-03, 1.46574468e-03, 
  7.15587299e-04, 3.33618583e-04, 1.55986763e-04, 9.66174445e-05
};

namespace sjsu::drivers {

 // arr = buffer
 // begin = pointer to the start of arr 
 float filter(std::array<float, sample_rate> arr, int begin) {
    float sum = 0;
    for (uint16_t i = 0; i < sample_rate; i++) {
      int idx = (begin + i) % sample_rate;
      sum += (arr[idx] * filter_const[i]);
    }
    return sum;
  }

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

  auto adc = resources::adc_0();
  std::array<float, sample_rate> data;  // store filter values
  data.fill(0);  // fill with 0s
  int begin = 0;

  hal::print<64>(*terminal, "Loop \n");
  while (true) {
    hal::delay(*counter, 500ms);

    as7341_sensor.set_smux(true);
    hal::delay(*counter, 300ms);
    std::array<hal::u16, 6> F1F4 = as7341_sensor.readAllChannels();

    as7341_sensor.set_smux(false);
    hal::delay(*counter, 300ms);
    std::array<hal::u16, 6> F5F8 = as7341_sensor.readAllChannels();

    data[begin] = adc->read();
    // CLEAR (ADC Count Range: 0 – 65535)
    hal::print<64>(*terminal, "CLEAR (F1F5): %u\n", filter(F1F4[4], begin));
    hal::print<64>(*terminal, "CLEAR (F5F8): %u\n", filter(F5F8[4]), begin);

    // 515nm & 555nm (ADC Count Range: 0 – 65535)
    hal::print<64>(*terminal, "F4 515nm: %u\n", filter(F1F4[3], begin));  // cyan
    hal::print<64>(*terminal, "F5 555nm: %u\n", filter(F5F8[0], begin));  // green

    hal::delay(*counter, 100ms);

    begin++;
    if (begin > sample_rate - 1) {
      begin = 0;
    }
  }
}
}  // namespace sjsu::drivers
#ifndef SAMD_Utils_h_
#define SAMD_Utils_h_

#include <Arduino.h>
#include <cstdint>

namespace samd_utils {

static const int GCLK_DIVSEL_DIRECT = 0;
static const int GCLK_DIVSEL_POW2 = 1;

namespace gclk {

uint32_t init(int clk_id, uint8_t clk_div, int clk_divsel, bool en_gclk_io = false);

void init_iopin_out(int iopin, int port_id = 0);
}

namespace adc {

void init(uint8_t ain_pin, uint16_t genclk_id, bool run_continuous = true, bool generate_interrupt = true,
          uint8_t adc_prescaler = 0, uint8_t adc_samplen = 0, uint8_t adc_samplenum = 0); 
void select_input(uint8_t ain_pin);
void start(bool wait = false);
inline uint32_t get_result() {
    uint32_t const result = ADC->RESULT.reg;
    return result;
}
void stop();
void reset();

void init_iopin_in(int iopin, int port_id = 0);
}

namespace dio {
// these emulate pinMode() for OUTPUT and INPUT using the IO pad & port
void init_iopin_out(int iopin, int port_id = 0);
void init_iopin_in(int iopin, int port_id = 0);

// these emulate digitalRead/digitalWrite using the IO pad & port
int iopin_read(int iopin, int port_id = 0);
void iopin_write( uint8_t val, int iopin, int port_id = 0); 
}

}


#endif

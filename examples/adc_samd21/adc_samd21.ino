#include <SAMD_Utils.h>

using namespace samd_utils; 

// Pre-requisites
// 1. Connect an input signal to the desired ADC pin depending on hardware.
//    a. simplest solution: connect to ground or 3V3
//    b. more interesting: connect to the wiper of a pot between 3V3 & ground

#if !defined(__SAMD21__)
#error Unsupported target
#endif

#if defined(ADAFRUIT_QTPY_M0)
  #define PIN_DREADY_STATE 4
#elif defined(ADAFRUIT_ITSYBITSY_M0)
  #define PIN_DREADY_STATE 15
#else
  #error Unsupported board for ADC configuration
#endif

volatile int dready_state;      // toggle, logic level on the output pin
volatile int adc_conv_count;    // count ADC conversions (average and report to Serial out)
volatile uint32_t adc_conv_sum; // sum of ADC conversions (average and report to Serial out)

struct adc_config_def 
{
  // conversion time 
  // T_conv = (7.0 + 0.5*N) * 2^(P+2) * D * Navg [CPU clock ticks]
  //   + N: samplen (0-63)
  //   + P: prescaler (0-7)
  //   + D: clock divider (0-255, treat as divsel=direct)

#if defined(ADAFRUIT_QTPY_M0)
  // GCLK
  static const uint8_t gclk_id = 5;
  static const uint8_t gclk_port = 0;
  static const uint8_t gclk_iopin = 11;

  // ADC
  static const uint8_t adc_port = 0;        // 0=PORTA, 1=PORTB
  static const uint8_t adc_iopin = 2;       // PxN, N = iopin, x = port 
  static const uint8_t adc_inputctrl_muxpos_ain = 0;    // AIN[x]

#elif defined(ADAFRUIT_ITSYBITSY_M0) 
  // GCLK
  static const uint8_t gclk_id = 3;
  static const uint8_t gclk_port = 0;
  static const uint8_t gclk_iopin = 17;

  // ADC
  static const uint8_t adc_port = 0;        // 0=PORTA, 1=PORTB
  static const uint8_t adc_iopin = 5;       // PxN, N = iopin, x = port 
  static const uint8_t adc_inputctrl_muxpos_ain = 5;    // AIN[x]
#endif

  static const uint8_t clk_div = 75;
  static const DIVSEL_T clk_divsel = GCLK_DIVSEL_DIRECT;
  
  static const uint8_t adc_prescaler = 0;   // 2^(0+2) = 4
  static const uint8_t adc_samplen = 6;     // 1[gain] + 12[bits]/2 + 6[samplen]/2 = 10
  static const uint8_t adc_samplenum = 3;   // 8x avg

  // 48000kHz / 75(clk div) / 4(prescaler) = 160kHz / 8(avg) / (3 samplen + 12/2 bits + 1 gain) = 2kSPS
} adc_config; 

// for SAMD21, use ADC_Handler (only one ADC), see samd21e18.h or samd21g18.h
void ADC_Handler()
{
  digitalWrite(PIN_DREADY_STATE, dready_state);
  dready_state = dready_state == 0 ? 1 : 0;

  adc_conv_count += 1;
  adc_conv_sum += (0x0FFF & ADC->RESULT.reg);

  ADC->INTFLAG.bit.RESRDY = 1;

}


void setup() {
  dready_state = 0;
  pinMode(PIN_DREADY_STATE, OUTPUT);

  // this will override the settings on GCLK3, used for XOSC32 in the Adafruit startup.c
  // but there is no 32kHz XO on the ItsyBisty, so assuming it's not used.
  gclk::init_iopin_out(adc_config.gclk_iopin, adc_config.gclk_port);  // PA17, GCLK[3] -- D13 on the ItsyBitsy
  gclk::init(adc_config.gclk_id, adc_config.clk_div, adc_config.clk_divsel, true);

  adc::init_iopin_in(adc_config.adc_iopin, adc_config.adc_port);
  adc::init(adc_config.adc_inputctrl_muxpos_ain, adc_config.gclk_id, 
    adc_config.adc_prescaler, adc_config.adc_samplen, adc_config.adc_samplenum);

  Serial.begin(115200);

  adc_conv_count = 0;
  adc_conv_sum = 0;
  adc::start();

}

void loop() {
  // put your main code here, to run repeatedly:
  if (adc_conv_count >= 2048) {
    uint32_t const result = adc_conv_sum >> 11;
    adc_conv_count = 0;
    adc_conv_sum = 0;
    Serial.println(result);
  }
}

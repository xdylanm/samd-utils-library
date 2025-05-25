#include "SAMD_Utils.h"

#if (defined(__SAMD21E18A__) || defined(__SAMD21G18A__)) && !defined(__SAMD21__)
#define __SAMD21__
#endif

#define FCLK_DFLL48M 48000000

namespace samd_utils {


////////////////  GCLK /////////////////////////////

namespace gclk {

// https://blog.thea.codes/understanding-the-sam-d21-clocks/
// packages/adafruit/hardware/samd/1.7.5/cores/arduino/startup.c

// Adafruit uses 
// GCLK0 - Main clock (120MHz from DPLL on SAMD51, 48MHz from DFLL on SAMD21)

// SAMD21
// GCLK1 - 32kHz XOSC (if populated?)

// SAMD51
// GCLK1 - 48MHz 
// GCLK2 - 100MHz 
// GCLK3 - 32kHz XOSC
// GCLK4 - 12MHz 
// GCLK5 - 1MHz 

/*!
  \brief Initialize GCLK from the CPU 48MHz DFLL with a specified clock divider

  \param clk_id the identifier for the GCLK generator (e.g. 5)
  \param clk_div the division factor, interpretation depends on clk_divsel (0-as scalar multiple, 1-as 2^x)
  \param clk_divsel the flag for the divisor type, 0 = scalar multiple, 1 = 2^x
  \param en_gclk_io DEBUG ONLY, requires init_pin_for_CLK_out and routes this GCLK to PA11
*/
uint32_t init(
  int const clk_id, 
  uint8_t const clk_div, 
  int const clk_divsel, 
  bool const en_gclk_io /*= false*/) 
{
  // input checking: can't bit shift 48000000 more than 24 bits and have > 0 fgclk
  // leave the clock disabled
  if ((clk_divsel == 1) && (clk_div > 24)) {
    return uint32_t(-1);
  }

#if defined(__SAMD21__)
  // start by disabling the GCLK
  GCLK->GENCTRL.reg = GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk_id);  // disable
  GCLK->CLKCTRL.reg = GCLK_GENCTRL_ID(clk_id);  //disable
  while (GCLK->STATUS.bit.SYNCBUSY);

  // set the divisor value
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(clk_id) | GCLK_GENDIV_DIV(clk_div);
  while (GCLK->STATUS.bit.SYNCBUSY);  

  // build the register value to enable the clock and set its source to DFLL48M (OE & DIVSEL handled separately)
  uint32_t reg_val = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk_id);

#elif defined(__SAMD51__)
  // build the register value to enable the clock and set its source to DFLL48M (OE & DIVSEL handled separately)
  uint32_t reg_val = GCLK_GENCTRL_DIV(clk_div) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL;

#else
#error Unsupported target
#endif

  // configure the generator of the generic clock, which is 48MHz clock; 
  // if DIVSEL = 0 --> fgclk = 48/div_factor MHz (or fgclk = 48MHz if div_factor = 0)
  // if DIVSEL = 1 --> fgclk = 48/(2 << div_factor) MHz
  if (clk_divsel == GCLK_DIVSEL_POW2) {
    reg_val |= GCLK_GENCTRL_DIVSEL;
  }
  if (en_gclk_io) {
    reg_val |= GCLK_GENCTRL_OE;
  } 

#if defined(__SAMD21__)
  GCLK->GENCTRL.reg = reg_val;    
  while (GCLK->STATUS.bit.SYNCBUSY);  
#elif defined(__SAMD51__)
  GCLK->GENCTRL[clk_id].reg = reg_val;
  while (GCLK->SYNCBUSY.reg & (1 << (clk_id + 2)));
#endif

  if (clk_divsel == GCLK_DIVSEL_DIRECT) {
    return clk_div > 1 ? (FCLK_DFLL48M / uint32_t(clk_div)) : FCLK_DFLL48M;
  } else if (clk_divsel == GCLK_DIVSEL_POW2) {
    return (FCLK_DFLL48M >> (clk_div+1));
  } 
  return uint32_t(-1);      // error otherwise


} 

/*!
  \brief Update the PORT configuration to mux the specified GCLK to the IO pin

  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)
  
  \note The specified IO pin must be support a connection to the GCLK. Refer to
    the table in the IO multiplexing considerations section of the datasheet.
*/
void init_iopin_out(int const iopin, int const port_id /*= 0*/)
{
  PORT->Group[port_id].DIRSET.reg = (1 << iopin);     // output PA11
  PORT->Group[port_id].PINCFG[iopin].bit.PMUXEN = 1;  // enable peripheral mux to define behaviour
#if defined(__SAMD21__)
  PORT->Group[port_id].PMUX[iopin >> 1].reg = iopin % 2 == 0 ? PORT_PMUX_PMUXE_H : PORT_PMUX_PMUXO_H;  // type H for GCLK
#elif defined(__SAMD51__)
  PORT->Group[port_id].PMUX[iopin >> 1].reg = iopin % 2 == 0 ? PORT_PMUX_PMUXE(0xC) : PORT_PMUX_PMUXO(0xC);  // type M for GCLK
#endif
}

} // namespace gclk

////////////////  ADC /////////////////////////////

namespace adc {

/*
  \brief Configure the ADC

  \param ain_pin Index of AIN corresponding to IO pin, e.g. PA02 maps to AIN[0] on SAMD21
  \param genclk_id The GCLK ID (see gclk::init)
  \param run_continuous Enable free-running mode (deafult true)
  \param generate_interrupt Enable interrupts on conversion complete (default true)
  \param adc_prescaler ADC pre-scaler code 0-7: divide clock by 2^(p+2) (default 0 = DIV4)
  \param adc_samplen Number of half-clock cycles to sample input for conversion 
    (SAMPCTRL.SAMPLEN, see section 33.6.5.1 in SAMD21 datasheet)
  \param adc_samplenum Number of conversions to average

  \pre The IO pin is configured for read to ADC (call adc::init_iopin_in)
  \pre The GCLK \a genclk_id is configured
  \pre The ADC is disabled (call adc::stop)

  \post The ADC is configured and enabled but not started

  \see https://blog.thea.codes/reading-analog-values-with-the-samd-adc/
  \see https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/

*/
void init(
    uint8_t ain_pin, 
    uint16_t const genclk_id, 
    bool const run_continuous /* = true*/,
    bool const generate_interrupt /* = true*/,
    uint8_t const adc_prescaler /* = 0*/, 
    uint8_t const adc_samplen /* = 0*/, 
    uint8_t const adc_samplenum /* = 0*/) 
{

#if defined(__SAMD21__)
  // select the generic clock generator used as clock source GCLK_CLKCTRL_GEN_GCLK5
  GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(genclk_id) | GCLK_CLKCTRL_ID_ADC);
  while (GCLK->STATUS.bit.SYNCBUSY);
#elif defined(__SAMD51__)
  static const int PCHCTRL_GCLK_ADC_ID = 40;
  GCLK->PCHCTRL[PCHCTRL_GCLK_ADC_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(genclk_id);
#endif

#if defined(__SAMD21__)
  // Select reference, internal VCC/2
  ADC->REFCTRL.reg |= ADC_REFCTRL_REFSEL_INTVCC1; // VDDANA/2, combine with gain DIV2 for full VCC range

  // Average control 
  uint16_t const ressel = adc_samplenum == 0 ? ADC_CTRLB_RESSEL_12BIT : ADC_CTRLB_RESSEL_16BIT;
  uint8_t const adjres = adc_samplenum <= 4 ? adc_samplenum : 4; 
  ADC->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(adjres) | ADC_AVGCTRL_SAMPLENUM(adc_samplenum);

  // Sampling time = (SAMPLEN+1)*(CLK_ADC/2), adds half clock-cycles
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(adc_samplen);
  
  // Input control: set gain to div by two so ADC has measurement range of VCC, no diff measurement so set neg to gnd, pos input set to pin AIN[i]
  // see MUXPOS[4:0] for more detail on AIN[i] pin selection
  ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | (0x0000001F & (uint32_t)ain_pin);
  while (ADC->STATUS.bit.SYNCBUSY);
  
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER(adc_prescaler) | ressel | (run_continuous ? ADC_CTRLB_FREERUN : 0); 
  while (ADC->STATUS.bit.SYNCBUSY);

  if (generate_interrupt) {
    // setup the interrupt
    ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable ADC interrupt on result ready
    while (ADC->STATUS.bit.SYNCBUSY);

    NVIC_SetPriority(ADC_IRQn, 3); //set priority of the interrupt
    NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts
  }

  ADC->CTRLA.bit.ENABLE = 1;

#elif defined(__SAMD51__)

  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER(adc_prescaler);   // not enabled yet

  // Select reference: VDDANA (may be different than 3V3 on ItsyBitsy)
  ADC0->REFCTRL.reg = ADC_REFCTRL_REFCOMP | ADC_REFCTRL_REFSEL_INTVCC1; // VDDANA
  while (ADC0->SYNCBUSY.bit.REFCTRL);
  
  // Average control 
  uint8_t const adjres = adc_samplenum <= 4 ? adc_samplenum : 4; 
  uint16_t const ressel = adc_samplenum == 0 ? ADC_CTRLB_RESSEL_12BIT : ADC_CTRLB_RESSEL_16BIT;
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(adjres) | ADC_AVGCTRL_SAMPLENUM(adc_samplenum);
  while (ADC0->SYNCBUSY.bit.AVGCTRL);

  // Sampling time = (SAMPLEN+1)*(CLK_ADC), cannot use >0 with offset comp
  ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(adc_samplen);
  while (ADC0->SYNCBUSY.bit.SAMPCTRL);

  // Input control: no diff measurement so set neg to gnd, pos input set to pin AIN[i]
  // see MUXPOS[4:0] for more detail on AIN[i] pin selection
  ADC0->INPUTCTRL.reg =  ADC_INPUTCTRL_MUXNEG_GND | (ADC_INPUTCTRL_MUXPOS_Msk & (uint32_t)ain_pin);
  while (ADC0->SYNCBUSY.bit.INPUTCTRL);

  ADC0->CTRLB.reg = ressel | (run_continuous ? ADC_CTRLB_FREERUN : 0); 
  while (ADC0->SYNCBUSY.bit.CTRLB);
  
  if (generate_interrupt) {
    // setup the interrupt
    ADC0->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable ADC interrupt on result ready
    
    NVIC_SetPriority(ADC0_1_IRQn, 3); //set priority of the interrupt
    NVIC_EnableIRQ(ADC0_1_IRQn); // enable ADC interrupts
  }

  ADC0->CTRLA.bit.ENABLE = 1;

#endif
  
}

void select_input(uint8_t ain_pin) 
{
#if defined(__SAMD21__)
  // Input control: set gain to div by two so ADC has measurement range of VCC, no diff measurement so set neg to gnd,
  // pos input set to pin AIN[i]. See MUXPOS[4:0] for more detail on AIN[i] pin selection
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | (0x0000001F & (uint32_t)ain_pin);
  while (ADC->STATUS.bit.SYNCBUSY);
#elif defined(__SAMD51__) 
  // Input control: no diff measurement so set neg to gnd, pos input set to pin AIN[i]
  // see MUXPOS[4:0] for more detail on AIN[i] pin selection
  ADC0->INPUTCTRL.reg =  ADC_INPUTCTRL_MUXNEG_GND | (ADC_INPUTCTRL_MUXPOS_Msk & (uint32_t)ain_pin);
  while (ADC0->SYNCBUSY.bit.INPUTCTRL);
#endif
}

/*! 
  \brief Start ADC conversion(s)
  
  If the ADC is configured in free-running mode and/or if interrupts are enabled, 
  this call should be non-blocking.

  \param wait Block until the conversion is complete before returning (default false)
*/
void start(bool wait /* = false*/) 
{
#if defined(__SAMD21__)
  
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->SWTRIG.reg = ADC_SWTRIG_START; // Start ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);    // Wait for sync

  if (wait) {
    while (ADC->INTFLAG.bit.RESRDY == 0);
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  // clear flag
  }

#elif defined(__SAMD51__)
  while (ADC0->SYNCBUSY.bit.ENABLE);
  
  ADC0->SWTRIG.reg = ADC_SWTRIG_START; // Start ADC conversion
  while(ADC0->SYNCBUSY.bit.SWTRIG);    // Wait for sync

  if (wait) {
    while (ADC0->INTFLAG.bit.RESRDY == 0);
    ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;  // clear flag
  }

#endif
}

/*! 
  \brief Stop the ADC by setting the enabled state to false.
*/
void stop() 
{
#if defined(__SAMD21__)
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);
#elif defined(__SAMD51__)
  ADC0->CTRLA.bit.ENABLE = 0;
  while (ADC0->SYNCBUSY.bit.ENABLE);
#endif
}

/*!
  \brief Reset the ADC by writing the software reset (SWRST) bit.
*/
void reset()
{
#if defined(__SAMD21__)
  ADC->CTRLA.bit.SWRST = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
#elif defined(__SAMD51__)
  ADC0->CTRLA.bit.SWRST = 1;
  while (ADC0->SYNCBUSY.bit.SWRST);
#endif
}

/*!
  \brief Set the IO pin as an input and update the PORT table mux configuration
    to connect the signal to the ADC

  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)

  \note The specified IO pin must be support a connection to the ADC. Refer to
    the table in the IO multiplexing considerations section of the datasheet.
*/
void init_iopin_in(int const iopin, int const port_id /*=0*/)
{
  PORT->Group[port_id].PINCFG[iopin].reg=(uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[port_id].DIRCLR.reg = (1 << iopin);     // input PA03
  PORT->Group[port_id].PINCFG[iopin].bit.PMUXEN = 1;  // enable peripheral mux to define behaviour
#if defined(__SAMD21__)
  PORT->Group[port_id].PMUX[iopin >> 1].reg = iopin % 2 == 0 ? PORT_PMUX_PMUXE_B : PORT_PMUX_PMUXO_B;  // type B for ADC, odd & even
#elif defined(__SAMD51__)
  PORT->Group[port_id].PMUX[iopin >> 1].reg = iopin % 2 == 0 ? PORT_PMUX_PMUXE(0x1) : PORT_PMUX_PMUXO(0x1);  // type B for ADC
#endif
}


} // namespace adc

////////////////// DIGITAL IO /////////////////////
namespace dio {

/*!
  \brief Set pin to digital out, initializes pin to 0 (like pinMode(OUTPUT))

  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)
*/
void init_iopin_out(int const iopin, int const port_id /*=0*/)
{
  PORT->Group[port_id].PINCFG[iopin].reg=(uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[port_id].DIRSET.reg = (1 << iopin);     // output PA07
}

/*!
  \brief Set pin to digital in (like pinMode(INPUT))

  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)
*/
void init_iopin_in(int const iopin, int const port_id /*=0*/)
{
  PORT->Group[port_id].PINCFG[iopin].reg=(uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[port_id].DIRCLR.reg = (1 << iopin);     // input at PA[iopin]
}

/*!
  \brief Read a binary value from an IO Pin (equivalent to digitalRead)

  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)

  \returns int 1 or 0
*/
int iopin_read(int iopin, int const port_id /*=0*/) 
{
  if ( (PORT->Group[port_id].IN.reg & (1ul << iopin)) != 0) {
    return 1;
  }
  return 0;
}

/*!
  \brief Write a binary value to an IO pin

  \param val 0 or 1 set the pin to logic low or high
  \param iopin The IO pin number (e.g. IO pin is 11 for PA11)
  \param port_id The PORT index (0=A, 1=B)
*/
void iopin_write(uint8_t val, int iopin, int const port_id /*=0*/) 
{
  uint32_t const pinmask = (1ul << iopin);
  if (val == 0) {
    PORT->Group[port_id].OUTCLR.reg = pinmask;
  } else {
    PORT->Group[port_id].OUTSET.reg = pinmask;
  }
}

} // namespace dio


} // namespace samd_utils

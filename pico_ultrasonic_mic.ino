#include "hardware/adc.h"

#define cpuClkKhz 125000 // data sheet says 133000 maximum
//#define ADCclkDiv1 96  // 500 kHz samplesrate
//#define ADCclkDiv 4800  // 10 kHz samplerate
//#define ADCclkDiv1 12000  // 4 kHz samplerate
#define ADCclkDiv1 192  // 250 kHz samplesrate

/* global variables */
char print_buffer[1024];

/* First we have any needed functions.  Setup and loop are at the end */

void adcSetClockRate(int ADCclkDiv){
  int adcEffectiveDivider ;
  int sampleRate ;
  if(ADCclkDiv<=96){
    adc_set_clkdiv (0) ; 
    adcEffectiveDivider=96 ; 
  } else { 
    adc_set_clkdiv (ADCclkDiv-1) ;
    adcEffectiveDivider=ADCclkDiv; 
  }
  uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
  // float f_clk_adc=48000.0 ;
  sampleRate=f_clk_adc*1e3/adcEffectiveDivider ;
  uint32_t adcReadBack=adc_hw->div ;
  float adcClkDivFrac=(adcReadBack & 0xFF)/256.0 ;
  uint16_t adcClkDivMSB=adcReadBack>>8 ;
  sprintf(print_buffer, "ADC sampleRate=%8.3f kHz\n",sampleRate/1e3);
  Serial.println(print_buffer);
  sprintf(print_buffer,"ADC readBack=%8xH adcClkDivMSB=%3d adcClkDivFrac=%8.3f\n",adcReadBack,adcClkDivMSB,adcClkDivFrac) ;
  Serial.println(print_buffer);
}

void adcInit(void) {
  // ADC is in an unknown state. We should start by resetting it
  reset_block(RESETS_RESET_ADC_BITS);
  unreset_block_wait(RESETS_RESET_ADC_BITS);
  // Now turn it back on. Staging of clock etc is handled internally
  adc_hw->cs = ADC_CS_EN_BITS;
  // Internal staging completes in a few cycles, but poll to be sure
  while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
    tight_loop_contents();
    }
  // Select ADC input 2 (GPIO28)
  // adc_gpio_init(28);
  // adc_select_input(2);

  // Select ADC input 1(GPIO27)
  adc_gpio_init(27);
  adc_select_input(1);

  // adc_set_clkdiv (ADCclkDiv) ;
  adcSetClockRate(ADCclkDiv1) ;
  adc_fifo_setup(true, false, 0, false, false);
  adc_run(true);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("ARISS Digital Ultrasonic Microphone");
  Serial.println("V1");

  /* Setup the ADC */
  set_sys_clock_khz(cpuClkKhz, true);
  adc_init(); // hardware API
   // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(27);
    // Select ADC input 0 (GPIO26)
    adc_select_input(1);


  //adcInit();
}

void loop() {
  /* Flash LED Twice for a reading */
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(850);                        // wait for a second

  /* Read the ADC */
  uint32_t result = adc_read();
  const float conversion_factor = 3.3f / (1 << 12);
    
  /* Calculate the data */

  /* Send the data */
  sprintf(print_buffer, "0x%03x -> %f V", result, result * conversion_factor);
  Serial.println(print_buffer);
//Serial.println("Data...");
}

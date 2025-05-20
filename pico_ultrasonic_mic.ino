#include "hardware/adc.h"
#include <math.h>
#include <stdio.h>

#define busyPin1 3
#define nFFT 64
#define DCremove 1
#define kHz 1e3

float fMin=0 ;
float fMax=0 ;
int nMean=0;
float fftRe[nFFT] ;
float fftIm[nFFT] ;
int kMin,kMax ;

#define cpuClkKhz 125000 // data sheet says 133000 maximum
//#define ADCclkDiv1 96  // 500 kHz samplesrate
//#define ADCclkDiv 4800  // 10 kHz samplerate
//#define ADCclkDiv1 12000  // 4 kHz samplerate
#define ADCclkDiv1 192  // 250 kHz samplesrate

/* global variables */
float sampleRate=0;
char stxt[128] ;
//#define nSamples 1024
#define nPreamble 16
int preambleBuffer[nPreamble] ;
int sampleBuffer[nFFT] ;

/* First we have any needed functions.  Setup and loop are at the end */
float sqr(float x){
  return x*x ;
}

float sampleBufferVolt(int k){
  float vv=(sampleBuffer[k]-2048)/4096.0*3.3 ;
  return vv ;
}

void adcSetClockRate(int ADCclkDiv){
  int adcEffectiveDivider ;
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
  sprintf(stxt, "ADC sampleRate=%8.3f kHz\n",sampleRate/1e3);
  Serial.println(stxt);
  sprintf(stxt,"ADC readBack=%8xH adcClkDivMSB=%3d adcClkDivFrac=%8.3f",adcReadBack,adcClkDivMSB,adcClkDivFrac) ;
  Serial.println(stxt);
}

void adcInit(void) {
  // // ADC is in an unknown state. We should start by resetting it
  // reset_block(RESETS_RESET_ADC_BITS);
  // unreset_block_wait(RESETS_RESET_ADC_BITS);
  // // Now turn it back on. Staging of clock etc is handled internally
  // adc_hw->cs = ADC_CS_EN_BITS;
  // // Internal staging completes in a few cycles, but poll to be sure
  // while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
  //   tight_loop_contents();
  //   }
  adc_init(); // hardware API

  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26); //27
  // Select ADC input 0 (GPIO26)
  // Select ADC input 1 (GPIO27)
  adc_select_input(0);

  adcSetClockRate(ADCclkDiv1) ;
  adc_fifo_setup(true, false, 0, false, false);
  adc_run(true);
}

void uart_putc1(char c){
  Serial.print(c) ;
}

void decout4(int k){
  // uart_putc1(32) ;
  uart_putc1( ((k/1000) % 10)+48 ) ;
  uart_putc1( ((k/100) % 10)+48 ) ;
  uart_putc1( ((k/10) % 10)+48 ) ;
  uart_putc1( (k % 10)+48 ) ;
  uart_putc1(',') ;
}  

void hexOut2(int k){
  // uart_putc1(32) ;
  sprintf(stxt,"%02x",k) ;
  uart_putc1( stxt[0]) ;
  uart_putc1( stxt[1]) ;
  uart_putc1(',') ;
}    

void hexOut3(int k){
  // uart_putc1(32) ;
  sprintf(stxt,"%03x",k) ;
  uart_putc1( stxt[0]) ;
  uart_putc1( stxt[1]) ;
  uart_putc1( stxt[2]) ;
  uart_putc1(',') ;
}      

void decout6(int k){
  // uart_putc1(32) ;
  uart_putc1( ((k/100000) % 10)+48 ) ;
  uart_putc1( ((k/10000) % 10)+48 ) ;
  uart_putc1( ((k/1000) % 10)+48 ) ;
  uart_putc1( ((k/100) % 10)+48 ) ;
  uart_putc1( ((k/10) % 10)+48 ) ;
  uart_putc1( (k % 10)+48 ) ;
  uart_putc1(',') ;
} 

void old_sendBuffer(){
  uart_putc1(13) ;
  uart_putc1(10) ;
  uart_putc1(13) ;
  uart_putc1(10) ;

  uart_putc1('S') ;
  uart_putc1('p') ;decout6(nPreamble) ;
  uart_putc1('n') ;decout6(nFFT) ;
  
  uart_putc1('d') ; decout6( ADCclkDiv1 ) ;
  uart_putc1('c') ; decout6( cpuClkKhz ) ;
  int chkSum=0 ;
  for(int k=0 ; k<nPreamble ; k++){
    int v=(preambleBuffer[k]) & 0xFFF ;
    chkSum+=v ;
    hexOut3(v) ;
    }

  for(int k=0 ; k<nFFT ; k++){
    int v=(sampleBuffer[k]) & 0xFFF ;
    chkSum+=v ;
    hexOut3(v) ;
    }
  decout4(chkSum) ;
  uart_putc1('s') ;
  uart_putc1(13) ;
  uart_putc1(10) ;
  sleep_ms(100) ;
}

void send_samples() {
  Serial.println("DATA:");
  for(int k=0 ; k<nFFT ; k++){
    Serial.println(sampleBuffer[k]);
    }
}

void send_spectrum() {
  Serial.println("FFT:");
  for(int k=0 ; k<=nFFT/2 ; k++){
      Serial.println(sqrt(sqr((float)fftRe[k])+sqr((float)fftIm[k])));
    }
}

float windowFactor=1.0 ;

/* Fast Fourier Transform from Elektor Magazine Pico SA Article Sept/Oct 2023
   by Prof. Martin Ossmann*/
void fft1(int flag , double scale ) {
    int j,k,l,m,m1,n1,r,r1,r2 ;
    double re,im,a,b,ru,rv,iu,iv,rs ;
     m1=nFFT / 2 ;
     n1=round(log(nFFT)/log(2)) ;
     //System.out.println(" n="+n+" n1="+n1) ;
     l=0 ;
     for ( k=0 ; k<nFFT ; k++){
       if ( k>l ) { 
         re=fftRe[l] ; 
         im=fftIm[l] ; 
         fftRe[l]=fftRe[k] ; 
         fftIm[l]=fftIm[k] ;
         fftRe[k]=re ; 
         fftIm[k]=im ;
         }
       j=m1 ;
       while( (j<=l) & (j>0) ){  l=l-j ; j=j / 2 ; }
       l=l+j ;
       }
     m1=1 ;
     for ( m=1 ; m<=n1 ; m++){
       a=cos(M_PI/m1) ; 
       b=flag*sin(M_PI/m1) ; 
       re=1 ; 
       im=0 ;
       for ( j=0 ; j<m1 ; j++){
       r=0 ;
        while ( r<=nFFT-1 ){ 
        r1=r+j ; r2=r1+m1 ;
        ru=fftRe[r1] ; iu=fftIm[r1] ;
        rv=fftRe[r2]*re-fftIm[r2]*im ;
        iv=fftRe[r2]*im+fftIm[r2]*re ;
        fftRe[r1]=ru+rv ; fftIm[r1]=iu+iv ;
        fftRe[r2]=ru-rv ; fftIm[r2]=iu-iv ;
        r=r+2*m1 ;
        }
      rs=a*re-b*im ; 
      im=a*im+b*re ;
      re=rs ;
      }
    m1=2*m1 ;
    }
    for( k=0 ; k<nFFT ; k++) {
      fftRe[k]=scale*fftRe[k] ; 
      fftIm[k]=scale*fftIm[k] ;
      //System.out.println(String.format(" re/im = %10.5f +im %10.5f",fftRe[k],fftIm[k])) ;
      }
    }

/* Blackman window function */
double windowFun(int k){
  double alpha=0.16 ;
  double a0=(1-alpha)/2 ;
  double a1=0.5 ;
  double a2=alpha/2 ;
  double w=a0-a1*cos(2*M_PI*k/(nFFT-1))+a2*cos(4*M_PI*k/(nFFT-1)) ;
  return w ;
  }
  

/* To compensate for the signal reduction by applying the window we divide the
   spectrum by the window factor.  This is the arithmetic mean of the window function values */
float getWindowFactor(){
  char ptxt[128];
    float mean1=0 ;
    for (int k=0; k<nFFT; k++) {
      mean1 += fabs(windowFun(k))  ;
      }
    mean1=mean1/nFFT ;  
    // sprintf(ptxt, "windowFactor=%8.3f\n",mean1) ;
    // Serial.println(ptxt);
    return mean1 ; 
    }


void windowedFFT(int flag , double scale ){
/*
  printf("FFT scale = %10.5f\n",scale) ;
  printf("windowed FFT input:\n") ;
    for(int k=8192 ; k<8212 ; k++){
      printf("k=%4d Re=%10.5f Im=%10.5f\n",k,fftRe[k],fftIm[k]) ;
      }
*/
  for(int k=0 ; k<nFFT ; k++){
    //double w=0.54-0.46*Math.cos(2*Math.PI*k/(nFFT-1)) ;
    //double w=0.5-0.5*cos(2*M_PI*k/(nFFT-1)) ;
    double w=windowFun(k) ;
    fftRe[k]=fftRe[k]*w;
    fftIm[k]=fftIm[k]*w;
    }
  fft1(flag ,scale/windowFactor) ;
  }

void spectrum_analysis() {
  for (int k=0; k<nFFT ; k++) {
    //fftRe[k]=cos(2*PI*k*64.5/nFFT)+0.01*random(1.0) ;
    // float frq=30e3 ;
    // fftRe[k]=cos(2*M_PI*k*frq/sampleRate) +0.001*( rand()%10-5) ;
    //frq=5.0 ;
    // fftRe[k]=cos(2*M_PI*k*frq/nFFT) +0.001*( rand()%10-5) ;
    fftRe[k]=sampleBufferVolt(k) ;
    // printf("k=%3d sampleBufferVolt=%8.3f\n",k,sampleBufferVolt(k)) ;
    fftIm[k]=0 ;
  }

  if(DCremove){
    float DCsum=0.0 ;
    for (int k=0; k<nFFT ; k++) { DCsum += fftRe[k] ; }
    DCsum=DCsum/nFFT ;
    for (int k=0; k<nFFT ; k++) { fftRe[k] -= DCsum ; }
  }

  windowedFFT(1,2.0/nFFT);

  float maxFFT=0.0 ;
  float maxFrq=0.0 ;
  int maxK=0 ;
  for(int k=kMin ; k<kMax ; k++){
     // printf("k=%4d FFTre=%8.3f   FFTim=%8.3f\n",k,fftRe[k],fftIm[k]) ;
    float v=sqrt(sqr((float)fftRe[k])+sqr((float)fftIm[k])) ;
    if(v>maxFFT){
      maxFFT=v ;
      maxFrq=((float) k) *sampleRate/((float)nFFT) ;
      maxK=k ;
      sprintf(stxt,"set maxK=%d maxFrq=%8.3f kHz   maxFFT=%8.3f\n",maxK,maxFrq/kHz,maxFFT) ;  
      Serial.println(stxt);
      sprintf(stxt,"fSample= %10.1f kHz",sampleRate/1e3) ;
      Serial.println(stxt);
      
    }
  }

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("ARISS Digital Ultrasonic Microphone");
  Serial.println("V1");
  /* Set the system clock to a known value */
  set_sys_clock_khz(cpuClkKhz, true);

  gpio_init(busyPin1);
  gpio_set_dir(busyPin1, GPIO_OUT);

  for (int k=0; k<nFFT ; k++) {
    fftRe[k]=0 ;
    fftIm[k]=0 ;
  }

  windowFactor=getWindowFactor() ;
 // nMean=0;  
  
  fMin=0 ;
  fMax=sampleRate/2 ;

  kMin=(int)(fMin/sampleRate*nFFT) ;
  kMax=(int)(fMax/sampleRate*nFFT) ;
  if(kMax>nFFT/2){ kMax=nFFT/2 ; }

  /* Setup the ADC */
  adcInit();
  /* Flash LED Twice for ready */
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                        // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

}

void loop() { 
  /// waitforDMA() ;
  adc_run(false);
  adc_fifo_drain();
  adc_run(true);

  /* TODO Not sure why we do this ..*/
  for(int k=0 ; k<100 ; k++){
    adc_fifo_get_blocking() ;
  }
  for(int k=0 ; k<nPreamble ; k++){
    preambleBuffer[k]=adc_fifo_get_blocking() ;
  }


  gpio_put(busyPin1, 1);
  for (int k = 0; k < nFFT; k++) {
    sampleBuffer[k] = adc_fifo_get_blocking();
    //  sampleBuffer[k]=k ;
  }
  gpio_put(busyPin1, 0);
  adc_run(false);
  send_samples();
  //spectrum_analysis();
  //send_spectrum();
  sleep_ms(10000) ;
}

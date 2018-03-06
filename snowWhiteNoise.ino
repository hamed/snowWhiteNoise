/**
 * @file
 * @author  Hamed Seyed-allaei <hamed@ipm.ir>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 */


/****************************************************************
      YOU DON'T NEED TO CHANGE THIS FILE FOR DAY TO DAY USES       
 ****************************************************************/


/**
 * Sampling Rate
 *
 * rc sets the sampling rates. Sampling Rate = 42MHz / rc.
 * Given rc = 5*42; the sampling rate will be 200KHz. 
 * The theoretical limit is 1MHz (The maximum frequency of ADC in arduino due), 
 * The practical limit probably is less, 
 * because you can do only 84 operation for each sample at best, 
 * and, this program needs to send 4 bytes to computer for every sample.
 */
unsigned int rc = 42*5;       // 200KHz  the sampling rate will be: 42000000/rc
unsigned int samplingRate=200000;
// the above variable are set from the host program. 
// no need to change them here.

/**
 * Number of Samples
 * 
 * total number of samples to be recorded. 
 * 2^22, four million samples, should give you a decent result.
 * at the current rate, 200K sample per second, it will take 20 seconds.
 */
unsigned long nSamples=1<<22;  // 1<<22 means 2^22 in alien language :-)


/** 
 * Buffer sizes, probably you don't need to touch it.
 *
 * We have two buffers, one for DAC, and one for ADC.
 * Each buffer is divided to NUMBER_OF_BUFFERS buffers 
 * of BUFFER_SIZE of 32 bits integers.
 * That is, NUMBER_OF_BUFFERS * BUFFER_SIZE * 4 bytes.
 * In this way, we can be sure each task has its own dedicated buffer. 
 * In some part of USB codes of arduino, 
 * I saw a 512 bytes, so I am going to set BUFFER_SIZE = 128, 
 * (The best reasoning you have ever heard!).
 * NUMBER_OF_BUFFERS, must be a power of 2, like 4, 8, 16, 32 or probably 64.
 * I choose 32, for NUM_OF_BUF, so we take 32KB of arduino's memory. 
 * I guess, it is fine to raise it to 64.
 * Every time we going to address a buffer we will do it in this way:
 * dacBuffer[dacIndex & divider] That's why NUMBER_OF_BUFFERS must be power of 2.
 */

#define BUFFER_SIZE 128
#define NUMBER_OF_BUFFERS 32

/* indexes to access dac buffer and adc buffer, 
 * each process, DAC, ADC, TRNG has its own index.
 * USB has two index, one for each buffer.
 */
unsigned long dacIndex=0, dacUSBIndex=0, trngIndex=0, trngidx=0; 
unsigned long adcIndex=0, adcUSBIndex=0;
unsigned long dacBuffer[NUMBER_OF_BUFFERS][BUFFER_SIZE];
unsigned long adcBuffer[NUMBER_OF_BUFFERS][BUFFER_SIZE];
const unsigned long divider=NUMBER_OF_BUFFERS-1;

#include <Scheduler.h>

void setup() {

  Serial.begin(9600);  // To print debugging messages.

  /************************************************   
   * Turning devices on.
   */
  pmc_enable_periph_clk(ID_TC0);  
  pmc_enable_periph_clk(ID_TC1);  
  pmc_enable_periph_clk(ID_TRNG); 
  pmc_enable_periph_clk(ID_DACC); 

  /************************************************
   * True Random Number Generator (cool!)
   *
   * TRNG generate one 32 bit random number each 84 master clock cycle,
   * which is 2Mhz of 16 bit random numbers, more than enough for us.
  */
  trng_enable(TRNG);  
  trng_enable_interrupt(TRNG);

  // It is good to have the timer 0 on PIN2, good for Debugging
  // You can connect it to oscilloscope or LED.
  int result = PIO_Configure( PIOB, PIO_PERIPH_B, PIO_PB25B_TIOA0, PIO_DEFAULT);

  /************************************************
   * Scheduler
   *
   * I love this library!
   * There are 2 loops: 
   * loopDAC sends DAC data to PC over USB.
   * loopADC sends ADC data to PC over USB. 
   */
  Scheduler.startLoop(loopDAC);  
  Scheduler.startLoop(loopADC);
}


unsigned long nDACUSBSamples=0, nADCUSBSamples=0;  // Number of samples to be written to USB.

/* Number of samples to be written to DAC and read from ADC.
 * 0 means there is no sample left.
 * -1 means that, no sample left and interrupts and clocks are stopped.
 */
signed long nDACSamples=-1, nADCSamples=-1;

void loopDAC() {
  if((nDACUSBSamples > 0) && (dacUSBIndex < trngIndex)) {
    SerialUSB.write((uint8_t *) dacBuffer[dacUSBIndex & divider], 4 * BUFFER_SIZE);  // Then write that buffer.
    nDACUSBSamples -= 2 * BUFFER_SIZE;
    dacUSBIndex++;  // and then go to the next buffer.
  }
    yield();  // Ok, we are done.
}

void loopADC() {
  if((nADCUSBSamples>0) && (adcUSBIndex < adcIndex)) {
    SerialUSB.write((uint8_t *) adcBuffer[adcUSBIndex & divider], 4 * BUFFER_SIZE);  // Then write that buffer.
    nADCUSBSamples -= 2 * BUFFER_SIZE;
    adcUSBIndex++;  // and then go to the nex buffer.
  }
    yield();
}
// Scheduler is cool, isn't it?


void loop() {
  /************************************************
  * Timer Counter
  *
  * We set 2 separate timers for each DAC and ADC devices: Channel 0 and 1 of TC0.
  * This timers are in the same block and synchronized. But their output signal is 
  * a bit different (TIOA0 and TIOA0).
  * TIOA1 controls DAC.
  * TIOA0 controls ADC.
  * ADC is half sampling time late, Because we want to measure the center of the signal,
  * generated by DAC.
  */
  TC_Configure(TC0, 1, TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_ACPA_SET|TC_CMR_ACPC_CLEAR|TC_CMR_ASWTRG_CLEAR|TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Configure(TC0, 0, TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_ACPA_CLEAR|TC_CMR_ACPC_SET|TC_CMR_ASWTRG_CLEAR|TC_CMR_TCCLKS_TIMER_CLOCK1);

  /************************************************
   * Digital to Analog Convertor
   * 
   * DAC, its DMA and its Interrupts are set here. 
   * I use pin DAC1 for output. 
   * I think I destroyed DAC0 while I was experimenting :-(
   * You are better to take care of this fragile beast!
   */
  dacc_reset(DACC);                 // Reset DACC registers
  dacc_set_writeprotect(DACC, 0);
  dacc_set_transfer_mode(DACC, 1);  // Full word transfer mode.
  dacc_set_power_save(DACC, 0, 0);  // sleep mode-0 (disabled), fast wakeup-0 (disabled)
  dacc_set_timing(DACC, 0x08, 1, DACC_MR_STARTUP_0); // refresh - 0x08 (1024*8 dacc clocks), max speed mode - 0 (disabled), startup time   - 0x10 (1024 dacc clocks) 
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02)|DACC_ACR_IBCTLCH1(0x02)|DACC_ACR_IBCTLDACCORE(0x01));  // Setting currents, I don't know much about it! any comment or helps is appereciated.
  dacc_set_channel_selection(DACC, 1);  // Select Channel 1 of DACC, (I just destroyed my channel 0 so this is my only choice.)
  DACC->DACC_MR |= DACC_MR_TRGEN;       // We want to use trigger.
  DACC->DACC_MR |= DACC_MR_TRGSEL(2);   // This triger is TIOA1
  DACC->DACC_IDR = ~(DACC_IDR_ENDTX);   // Disabling Interrupts.
  DACC->DACC_IER = DACC_IER_ENDTX;      // Enabling Interrupts.
  DACC->DACC_PTCR = DACC_PTCR_TXTEN | DACC_PTCR_RXTDIS;
  DACC->DACC_CHER = 2;  // enable channel 1. for channel 0 use 1


  /************************************************
   * Analog to Digital Convertor
   * 
   * ADC, its DMA and its Interrupts are set here.
   * It records from pin A7.
   * 
   * ADC works in half-word mode, The buffer is 32 bits. 
   * That is why you see 2*BUFFER_SIZE as buffer counter.
   * the below code is taken from adc_init(ADC, SystemCoreClock, 
   * ADC_FREQ_MAX, ADC_STARTUP_FAST);
   */
  ADC->ADC_CR = ADC_CR_SWRST;         // Reset the controller.
  ADC->ADC_MR = 0;                    // Reset Mode Register.	
  ADC->ADC_PTCR = (ADC_PTCR_RXTDIS|ADC_PTCR_TXTDIS);  // Reset PDC transfer.

  ADC->ADC_MR |= ADC_MR_PRESCAL(3);   // ADCclock = MSCK/((PRESCAL+1)*2), 13 -> 750000 Sps
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT0; // What is this by the way?
  ADC->ADC_MR |= ADC_MR_TRACKTIM(15);
  ADC->ADC_MR |= ADC_MR_TRANSFER(1);
  ADC->ADC_MR |= ADC_MR_TRGEN_EN; 
  ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG1; // selecting TIOA0 as trigger.
  ADC->ADC_CHER= ADC_CHER_CH0;  /* A7 Channel enabling register. 
                                  * ADC_CHER_CH0? I spend a couple of days to find that!
                                  */
  /* Interupts */
  ADC->ADC_IDR   = ~ADC_IDR_ENDRX;
  ADC->ADC_IER   =  ADC_IER_ENDRX; 

  /************************************************ 
   * We set USB here. We wait until it is ready.
   */
  Serial.print("Setting up USB connection to host ");
  SerialUSB.begin(0); // Open USB.
  while(!SerialUSB) { // Wait for the host program.
    delay(1000);
    Serial.print(".");
  };  
  Serial.println(" done");
  
  // Reading nSamples and samplingRate.
  Serial.print("Number of Samples is: ");
  while(SerialUSB.available() == 0);  // Wait for some inputs
  nSamples = SerialUSB.parseInt();    // Read the input as integer
  Serial.println(nSamples);
   
  Serial.print("Sampling Rate is: ");
  while(SerialUSB.available() == 0);    // Wait for some other inputs
  samplingRate = SerialUSB.parseInt();  // Read some other input as integer
  Serial.print(samplingRate);
  Serial.println(" Hz");
    
  while(SerialUSB.available() > 0) {SerialUSB.read();} // Make sure usb buffer is empty.

  rc = 42000000/samplingRate;

/* 
 * Timings
 *
 * To see the following figure properly, you should use mono space font.
 *
  A      C  A      C  A      C  A      C  A      C  A      C  A
__|______|__|______|__|______|__|______|__|______|__|______|__|_    TC1
   ______    ______    ______    ______    ______    ______    _ 
__|      |__|      |__|      |__|      |__|      |__|      |__|     TIOA1   
     _________                     _________
    |         |_________          |         |          _________
____|                   |_________|         |_________|         |   DAC1
          _         _         _         _         _         _
_________| |_______| |_______| |_______| |_______| |_______| |___   ADC clock
         |-|<- ADC sample and hold period.
          ____      ____      ____      ____      ____      ____
_________|    |____|    |____|    |____|    |____|    |____|    |   TIOA0

         C    A    C    A    C    A    C    A    C    A    C    A    
_________|____|____|____|____|____|____|____|____|____|____|____|   TC0
*/
  
  TC_SetRC(TC0, 1, rc);      // TIOA1 goes LOW  on RC.
  TC_SetRA(TC0, 1, rc/2-25); /* TIOA1 goes HIGH on RA. DAC starts on TIOA rising, 
                              * and its result is ready after 25 clocks, 
                              * so we start 25 clocks earlier.
                              */
  /*  timing of ADC */
  TC_SetRC(TC0, 0, rc);      // TIOA0 goes HIGH on RC.
  TC_SetRA(TC0, 0, rc/2);    /* TIOA0 goes LOW  on RA. 
                              * DACC send out the signal after 25 clocks, 
                              * and we want to measure the center (rc/2).
                              */
 
  dacIndex=0; dacUSBIndex=0; adcIndex=0; adcUSBIndex=0; trngidx=0; trngIndex=0;

  nDACUSBSamples = nSamples;  
  nADCUSBSamples = nSamples;

  nDACSamples = nSamples;
  nADCSamples = nSamples;

  Serial.print("Filling DAC buffers with random numbers ");
  NVIC_EnableIRQ(TRNG_IRQn);
  while((trngIndex-dacIndex) <= divider) {delay(1); Serial.print('.');}
  Serial.println(" done");

  Serial.print("Sending random numbers to host ");
  while((dacUSBIndex < trngIndex) && nDACUSBSamples) {delay(1); Serial.print('.');}
  Serial.println(" done");
  
  Serial.print("Setting up DAC DMA buffer ...");
  DACC->DACC_TPR  = (unsigned long) dacBuffer[dacIndex & divider];  // DMA buffer
  DACC->DACC_TCR  = (unsigned int)  BUFFER_SIZE; // DMA buffer counter
  DACC->DACC_TNPR = (unsigned long) dacBuffer[(dacIndex + 1) & divider];  // next DMA buffer
  DACC->DACC_TNCR = (unsigned int)  BUFFER_SIZE; // next DMA buffer counter
  nDACSamples -= 2 * BUFFER_SIZE;
  Serial.println(" done");

  Serial.print("Setting up ADC DMA buffer ...");
  ADC->ADC_RPR  = (unsigned long) adcBuffer[adcIndex & divider];  // DMA buffer
  ADC->ADC_RCR  = (unsigned int)  2 * BUFFER_SIZE;  // ADC works in half-word mode.
  ADC->ADC_RNPR = (unsigned long) adcBuffer[(adcIndex + 1) & divider];  // next DMA buffer
  ADC->ADC_RNCR = (unsigned int)  2 * BUFFER_SIZE;
  nADCSamples -= 2 * BUFFER_SIZE;
  Serial.println(" done");


  Serial.print("Enabling DACC and ADC interrupts ...");
  NVIC_EnableIRQ(DACC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR  =  ADC_PTCR_RXTEN;  // Enable receiving data.
  ADC->ADC_CR   |=  ADC_CR_START;    //start waiting for trigger.
  Serial.println(" done");
  
  /************************************************
  * We did a great job up to now. 
  * now we rest for a couple of  second and then we start.
  */

  Serial.print("Writing to DAC, reading from ADC and  Sending to host ...");

  TC0->TC_CHANNEL[1].TC_SR;  
  TC0->TC_CHANNEL[0].TC_SR;
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;  
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;  

  TC0->TC_BCR = TC_BCR_SYNC;  // Start the clocks.   3, 2, 1 and GO!
  while((nADCSamples > -1) || (nDACSamples > -1)) {yield();} // If you have got something to do, do it.
  Serial.println(" done");
  
  Serial.print("Sending tail of buffers to host ... ");
  while((nADCUSBSamples > 0) || (nDACUSBSamples > 0)) {yield();} // If you have got something to do, do it.
  Serial.println(" done");

  SerialUSB.end(); Serial.println("Closing open USB connections ... done");
  Serial.println("Now go and check the data on host software.");
  yield();
}


void TRNG_Handler() {  // called every 84 master clock.
/**
 * True Random Number Generator
 *
 * It generates a 32 bits random number per 84 master clock cycle. 
 * Each time a random number is ready, this interrupt is called.
 * We can directly put that 32 bit in buffer (viva little endian!), 
 * but, we can do better than that. 
 * DAC only use 12 less significant bits. We use one of this 4 abundant bits as a tag.
 * This tag will help us to discriminate between data send by DAC or ADC on PC. 
 * So, any 16 bit integer with 1 on its most significant is originated from trng and dac.
 * The rest are values read by ADC.
 */
  int trngISR = TRNG->TRNG_ISR;
  if((trngIndex-dacIndex) <= divider) {  // TRNG! Stop! DAC is using dacBuffer[dacIndex & divider], dont mess with it.
    if(trngISR) {
      dacBuffer[trngIndex & divider][trngidx] = (0x0FFF0FFF & TRNG->TRNG_ODATA) | 0x80008000; // TRNG, put 32 bits of random numbers into the buffer.       
      //dacBuffer[trngIndex & divider][trngidx] =  0x8FFF8000; // This will give a square wave, good for debugging.
      trngidx++;  
      if(trngidx == BUFFER_SIZE) {
        trngidx=0;
        trngIndex++;
      }
    }
  }
}


void DACC_Handler() {
/**
 * This interrupt is called when the next buffer counter is zero.
 * That means, we have finished with the current buffer, 
 * dacIndex points to the current buffer, so we increment it by one,
 * so it will point to the new current buffer.
 * then we set the value of the next buffer (dacIndex+1).
 */
  unsigned long status =  DACC->DACC_ISR;
  if(status & DACC_ISR_ENDTX) {
    if(nDACSamples > 0) {
      dacIndex++;  // Move dacIndex to the current buffer.
      DACC->DACC_TNPR = (unsigned long) dacBuffer[(dacIndex+1) & divider]; // Set the next buffer.
      DACC->DACC_TNCR = BUFFER_SIZE;  // set the next buffer counter.
      nDACSamples -= 2 * BUFFER_SIZE;
    }
    if((status & DACC_ISR_TXBUFE) && (nDACSamples == 0))  {

      nDACSamples=-1;
      dacIndex += 2;  // Set the buffer index for the next run.
                      // and the data will be written up to this
      NVIC_DisableIRQ(DACC_IRQn);  // Disable Interrupt
      NVIC_DisableIRQ(TRNG_IRQn);  // No need to produce random number.
      TC_Stop(TC0,1);
    }
  }
}


void ADC_Handler() {                 // move DMA pointers to next buffer
/**
 * See DACC_Handler(), they are similar.
 */
  unsigned long status = ADC->ADC_ISR;
  if(status & ADC_ISR_ENDRX)  {
    if(nADCSamples > 0) {
      adcIndex++;  // this is current buffer (PCR), next buffer PNCR is adcIndex+1
      ADC->ADC_RNPR  = (unsigned long) adcBuffer[(adcIndex+1) & divider];

      // This is tricky! each time, DMA send 16 bits to dacs, 
      // but the buffer size is 32*BUFFER_SIZE.
      ADC->ADC_RNCR  = 2 * BUFFER_SIZE;
      nADCSamples   -= 2 * BUFFER_SIZE;
    }
    if((status & ADC_ISR_RXBUFF) && (nADCSamples == 0)) {
      nADCSamples=-1;   
      adcIndex +=2;    // Set the buffer index for the next run.
                       // and the data will be written up to this 
      NVIC_DisableIRQ(ADC_IRQn);  // Disable Interrupt
      TC_Stop(TC0,0);
      adc_stop(ADC);      
    }
  }
}

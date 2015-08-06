/*
Copyright (c) 2014 - ngg123 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

/* msp430.h will use the compiler flags to figure out which chip we are targeting
   This will #include the correct msp430x2yyy.h
 */

#include <msp430.h> 
#include <limits.h>
#include "softUART-t2.h"

#define		CPU_DUTYCYCLE

#define     RED_LED               BIT0
#define     GRN_LED               ( BIT3)
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT
/*
 The following analog inputs cannot be used on the LaunchPad:
 A0 -- used for TA0CLK input (local CXO) (this is universal, not just with LP)
 A1 -- probably can be used, but lots of 4.8 MHz noise
 A2 -- unless RX jumper is removed, LP UART pulls this up to Vcc
 A3 -- Switch 2 is pulled up by external resistor
 A6 -- LED2 pulls this down to ground, also used by softUART
 A7 -- used by softUART
 */
#define     WWVB_ANT_CHANNEL      4
#define     INCH_WWVB             (INCH_1*WWVB_ANT_CHANNEL)

// these are used to set the source of the SMClk
#define     WWVB_LO_DCO           0
#define     WWVB_LO_OCXO          SELS 
#define     ADC_BLOCK_SIZE        8
#define     ADC_TRIGGER           SHS_2
#define     INITIAL_TIMER_RATE    10 //40 with DCO=6, 44 with DCO=7
#define     PHASE_ADJUSTMENT      1 // amount by which dS modulator nudges the phase

//register unsigned int volatile *TAIV_PTR asm("r4");

// This is the main DSP part of the program.  Each element of the array holds the ADC
// value from one of 4 phases of the carrier (0, 90, 180, 270).  The ADC ISR will
// manipulate both the data in the phaseData array and the value of phaseNum pointer
// (to keep track of which phase is current)

unsigned int volatile adc_buffer[(2*ADC_BLOCK_SIZE)];
//unsigned int volatile cycles_measured = 0;
unsigned int buffers_read = 0;
unsigned int  phaseData[4] = {0,0,0,0};

/* timer_rate is the amount of phase the timer should accumulate before triggering the
   next ADC conversion.
   Using an external 4.8MHz crystal, timer_rate = 10 means we sample every cycle
*/
register int timer_rate asm("r5");
register int official_ccr asm("r6");

void initClocks(void) {
  /*
    We want to run the CPU (main clock) as fast as practicable (SELM=0, DIVM=0).  
    The G series datasheets specify a maximum recommended frequency as 16MHz, 
    but it seems to run reliably at 21MHz with the standard Launchpad supply 
    voltage of 3.6V at room temperature, which is as fast as the DCO will go 
    (RSEL = 15, DCO = 7) under those conditions.  
    
    We want to run the TimerA from the sub-main clock at a frequency that can be 
    easily divided into 480kHz (the toggle rate for the ADC trigger, to get a 
    240kHz sample rate).  When the receiver is done, the SMCLK should be sourced 
    from LFXT1CLK (SELS=1, DIVS=0) which should be sourced from
    an external, stable oscillator connected to the XT1 source (LFXT1S =3, XCAP=0 )
    (note, that the G series deviced can't drive a crystal at this frequency, 
    so a real oscillator is needed). 

    The DCO isn't super-stable... Even with MODx = 0, the 1 sec average frequency
    has a std deviation of around 150-250 ppm.
   */
  BCSCTL3 = LFXT1S_2 | XCAP_0; // LFXT = digital external clock, minimal capacitance
  BCSCTL2 = SELM_0 | DIVM_0 | WWVB_LO_DCO; // MCLK = DCO, no division, SMCLK = WWVB_LO

  /*
    DCO For one device (room temp, RSEL=15,MOD=0): 
    7 (0xE0)= 21.4 MHz
    6 (0xC0)= 19.3 MHz
    5 (0xA0)= 17.8 MHz
    4 (0x80)= 16.6 MHz
  */
  DCOCTL = 0xC0 | 0x07;
  
  BCSCTL1 = 13; // workaround for BCL12 errata in 430g2x31 (and most G series devs)
  BCSCTL1 = 15;
  //P2SEL = BIT6 | BIT7; // workaround for BCL14 errata in 430g2x31 
  
}

void initTimer(void){
  /* 
     Set up Timer A as a trigger source for the ADC.
     
     CCR0 will be used to trigger the ADC because it has a faster and
     higher priority interrupt than CCR1 (and CCR2).  The timer should use
     continuous mode (MC = 2), not up mode.  

     The trade-off is that we would get "free" ADC triggering (no interrupts
     except for the ADC block-done interrupt) but TACCR1 would never
     match for values greater than about 40.  So, any (lower priority)
     thing that wanted to take a timed interrupt would need to keep count
     in a fast-executing CCR1 ISR (which is intrinsically slower to execute
     and has a lower interrupt priority), and handle a lot of interrupts, and 
     CCR1 could never measure a capture value > 40, which makes it 
     pretty much useless.  In the current form, we add timer_rate to 
     TACCR0 in the fast, high-priority TA0 ISR, which leave CCR1 free for 
     capturing external events or generating periodic interrupts (or PWM).

     The timer source is set by TASSEL.  We use the TA clock (SEL=0),
     which should be source from a highly-stable local timebase (like an
     OCXO) because it is the ultimate source for our ADC sampling trigger.

     Page 4 of this file has a nice circuit for a Colpitts oscillator for 
     a 4.8MHz fundamental mode crystal: 
     http://www.axtal.com/data/publ/ukw1979_e.pdf

     TACCR0 is set for compare mode (CM_0, CCIS=2) with interrupts (CCIE)
     on match and the output modulator is set to toggle on match (OUTMOD=4).
     Note that the ADC triggers on one edge only, so the ADC trigger rate is
     half the rate implied by the timer_rate variable.

     TACCR1 is set for compare mode with the output modulator turned off
     (OUTMOD_0) with interrupts turned off.  It is available for use by
     other things.

     
  */

  TACCTL0 = CM_0 | CCIS_2 | CCIE | OUTMOD_4; //set up compare mode
  TACCTL1 = CM_0 | CCIS_2 | OUTMOD_0;
  TACTL = TASSEL_0 | MC_2 | TACLR ; //ext TACLK, continuous mode, clear TAR
  // initialize the TACCR's to 0
  TACCR0 = 0;
  TACCR1 = 0;
  /*
    This is for testing only: The output modulators can be routed to
    Port 1 pins by setting P1SEL bits (to select the CCR as source) and
    setting the port direction to out.  
  */
  P1SEL |= BIT0; // set TACLK for TA peripheral use
  //P1DIR |= BIT5;  // set TA0.1 as output
  P1DIR &= ~BIT0; // set TACLK as input
  
  // Initialize the timer_rate to something reasonable
  timer_rate = INITIAL_TIMER_RATE;
}

void initADC(void) {
  /*
    This is where we set up the ADC10 peripheral to sample data and transfer it to
    memory with the DTC (baby DMA).  
  */

  /*
    We need to enable analog input on the pin the signal is connected to.  ADC10AE0
    bits 0 through 7 correspond to the A0 through A7 mux inputs.  ADC10AE1 enables
    A12 through A15 on bits 4 through 7, but G-series targets don't implement these
    mux inputs.

    For most targets, PxSEL and PxDIR are "don't cares" for selecting analog inputs.
    PxREN will still turn on the internal weak pull up/down resistor, and PxOUT
    still controls the polarity of the voltage applied to the resistor.
  */
  ADC10AE0 = (1 << WWVB_ANT_CHANNEL);
  P1OUT &= ~(1 << WWVB_ANT_CHANNEL);
  P1REN &= ~(1 << WWVB_ANT_CHANNEL);

  
  // We must disable the ADC before changing anything 
  ADC10CTL0 &= ~ENC;

  /*
    Set up the Data Transfer Controller (DTC) to move conversion results to a block
    of memory in RAM.  We want Two-block mode to double buffer the input (reduces
    criticality of timing the main loop to deal with ADC10 Interrupts) and
    continuous transfer, so the DTC automatically starts filling the first buffer
    again after finishing the second one.

    The block size parameter sets the size of each block, so in two-block mode 
    we need to allocate double this space as a contiguous region in memory.
  */
  ADC10DTC0 = ADC10TB + ADC10CT; // 2-block mode & continuous transfer
  ADC10DTC1 = ADC_BLOCK_SIZE; // set block size
  ADC10SA = (unsigned int) &adc_buffer; // set the starting address of the buffer

  /* 
     Set up and turn on the ADC itself.  
     We want to use the internal 1.5V reference generator to get maximum resolution
     (SREF = 1, REFON = 1, REF2_5V = 0).  We want the generator on continuously, 
     not output externally, and set for  high speed sampling (so SR, REFOUT, REFBURST = 0).  
     We want sampling triggered by the CCR0, so MSC = 0. 
     
     For max sample rate and temporal resolution, we want the minimum sampling 
     period (SHT_0). Clock source set by SSEL bits (SSEL_0 = ADC10OSC, 3..6MHz 
     internal oscillator) and divided by the DIV bits (DIV_0 = /1). Input channel
     is selected by the INCH bits, and should match analog input we enabled above. 

     Note: The data sheet for the target will give the maximum value for
     SSEL_clock / DIV, but it is usually around 6.3 MHz.  The G-series user guide
     and device data sheets suggest that the ADC may still operate but with poorer 
     linearity when the clock rate is higher (but this hasn't been tested).  
    
     CONSEQ_2 sets repeat single channel mode (each triggers causes a conversion 
     on one channel). The trigger souce is selected by the SHS bits (the source 
     of the trigger selected by each SHS value varies widely between targets).     
  */
  
  ADC10CTL0 = SREF_1 | REFON | ADC10SHT_0 |  ADC10ON;
  ADC10CTL1 = INCH_10 | SHS_2 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_2;
  //ADC10CTL1 = INCH_WWVB | SHS_2 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_2;
  /* 
     Delay to allow the ADC to warm up (trying to use the ADC before it's
     ready causes it to become non-responsive until we set it up again. 
     The documentation suggests that we don't need to delay as long as we
     do, but experience suggests otherwise.  This initialization function
     only gets run occaisonally, so being conservative here isn't 
     a big problem.
  */

  __delay_cycles(10000);
  // Enable ADC10 interrupts (will trigger on DTC completion)
  ADC10CTL0 |= ADC10IE;
  // Arm ADC10 (will trigger on CCR0 match)
  ADC10CTL0 |=ENC;
}




int main(void) {
  int i;
  int ms = 0; // millisecond counter
  int tms = 0; // ten millisecond counter
  int hms = 0; // hundred millisecond counter
  long longPhase[4] = {0,0,0,0};
  

  /*
    These variables are used by the delta-sigma modulator.
    -sigma is the modulator's integrator
    -calRate is the input to the modulator, and also the
    term added to the integrator when the last bitstream
    output was -1
    -icalRate is the therm added to the integrator when
    the last bitstream output was +1
    -delta is the bitstream output

    Setting calRate to INT_MAX will add an extra TACLK cycle
    worth of phase to the NCO once every 8 samples. INT_MIN
    will subtract one TACLK every 8 samples.  This represents
    an adjustment range of around +/-0.3%  (A clock running
    too fast is corrected with a positive calRate value)

    As a result, the adjustment granularity is much smaller 
    than the clock phase noise. We can usually trim the DCO 
    to within about 30 ppm using this method, but the DCO
    wanders by around +/-150 ppm over a minute.

    Remaining phase noise in the PPS signal after d-S 
    modulation is around 400 ppb, pk-pk.
   */
  int sigma = 0;
  int deltaCal = 29; // +25 for  +10 ppm offset
  int calRate = +16384 - deltaCal; // center is at 16384
  int icalRate = calRate - INT_MIN; // ~ -16384



  unsigned int  tx_buf;
  unsigned char rx_buf;
  int state;
#ifndef   SOFTUART_INTERNAL_TX_TIMER
  int tx_timer = 0;
#endif

  // Stop the watchdog
  WDTCTL = WDTPW |  WDTHOLD;
  // enable output for pins connected to LEDs on Launchpad
  P1DIR |=  GRN_LED;
  // turn off LEDs
  P1OUT &= ~(GRN_LED);
  // init the peripherals
  initClocks();	
  initADC();
  initTimer();

  // enable interrupts
  __bis_status_register(GIE);
  

  /*
    This is the main loop in the program.  The loop sleeps on
    every iteration.  The CPU is restarted by the ADC10 ISR,
	which fires every time a block of data has been acquired. 

    The ADC runs on a strict timer from an external CXO, so 
    this loop runs on a fixed schedule (240 kHz / ADC_BLOCK_SIZE).
    That makes it very easy to do things like a soft UART.
    
    
  */

  P1DIR |= BIT5;
  initTX();
  initRX(&state, &rx_buf);
  tx_buf = startTX('h');

  while(1) {
    

    state = doRX(state, &rx_buf);
    if (10 == state){
      if (0xFFFF == tx_buf){
      // This isn't a good way to check for tx_complete, but it's good enough
	  tx_buf = startTX(rx_buf);
      }
    }
#ifndef   SOFTUART_INTERNAL_TX_TIMER
	tx_timer++;
    if (3 == tx_timer){
      tx_timer =0;
#endif
      tx_buf = doTXbit(tx_buf);
#ifndef   SOFTUART_INTERNAL_TX_TIMER
      }
#endif


#ifdef CPU_DUTYCYCLE
    P1OUT ^= BIT5;
    // sleep until the ADC10 DTC finishes a block
    __bis_status_register(CPUOFF);
    P1OUT |= BIT5;
#else
	__bis_status_register(CPUOFF);
#endif

    /*
      This is our PPS signal, and we want it as close to the CPU wakeup
      possible to avoid being delayed by interrupts.
     */
    if (500 == ms){
      P1OUT ^=GRN_LED;
      ms = 0;
    }
    /*
      This is a very simple 1st-order delta-sigma modulator 
      which is used to fine-tune the NCO frequency by 
      nudging the phase up and down.  

      The purpose of this is just to get the LO close to the
      nominal WWVB frequency so the PLL can lock.

      We don't use extended assembly here, because GCC just
      takes it as permission to ignore what we're trying to do.
    */
    if(sigma < 0){
      sigma +=icalRate;
      asm("add #-1, r6"); // softCCR -= 1
    } else {
      sigma +=calRate;
      asm("add #1, r6"); // softCCR += 1;
    }
    

    /*
      First we accumulate each buffer of ADC samples in a structure
      called phaseData (each element of the array represents on of the
      four phases we measure per cycle). gcc doesn't like to generate
      efficient code for adding a volatile int to a long, so that is
      part of the reason for this two-step process.
      
      Errata: The below code implicitly assume that ADC_BLOCK_SIZE=8,
      even though it's sorta kidna written as if it didn't assume that.
     */
    if (0 == (ADC10DTC0 & ADC10B1)){
      for (i = 0; i < ADC_BLOCK_SIZE/2; i++){
	phaseData[i] += adc_buffer[i];
	phaseData[i] += adc_buffer[i*2];
      }
    }else{
      for (i = 0; i < ADC_BLOCK_SIZE/2; i++){
	phaseData[i] += adc_buffer[i+ADC_BLOCK_SIZE];
	phaseData[i] += adc_buffer[i*2 +ADC_BLOCK_SIZE];
      }
    }

    

    /*
      ADC10 data is 10 bits long, so we will overflow the phaseData 
      after adding 2^(16-10)=64 samples to each phase.  Since each
      buffer has two samples for each phase, we need to add phaseData
      to our long after 32 blocks have been processed to avoid overflowing
      the ints.  At this point we have processed 60 cycles of the carrier,
      which represents (60/(60 kHz)) = 1 ms of time.
    */
    buffers_read++;
    if (30 == buffers_read){
      //P1OUT ^= GRN_LED;
      buffers_read = 0;
      for (i = 0; i<4;i++){
	longPhase[i] += phaseData[i];
	phaseData[i] = 0;
      }
      ms++;
      tms++;
      hms++;
    }

    /*
      This is the heart of the Costas Loop.  The basic idea here is the
      same as with a regular PLL, namely that we want to 
      measure the phase difference between our LO and the reference,
      which in this case is WWVB.  longPhase[0] and [2] represent the
      in-phase (I) component of the reference, so if the LO is exactly
      in phase with the reference, longPhase[0] - longPhase[2] = 0.
      What makes the costas loop different is that we multiply the 
      phase offset by the sign of the quadrature (Q) component of the
      reference.  This is because the ref is allowed to arbitrarilly 
      change phase by 180 degrees (due to the BPSK modulation). 
    */
    

    if (99 == hms){
      
      long dPhase;
      // First check if the in-phase leg is pos or neg
      if (longPhase[1] > longPhase[3]){
	dPhase = longPhase[0] - longPhase[2];
	
      } else {
	dPhase = longPhase[2] - longPhase[0];
	
      }
      /*
	When the phase is positive, it means our LO slipped backward a
	little and we are sampling the reference too late.
       */
      if (dPhase > 0){
	
	// In here we should do something to speed up the LO
	
      } else {
	// In here we should do somethign to slow down the LO
	
      }

      for (i=0;i<4;i++){
	longPhase[i]=0;
      }

    } 
    if (100 == hms) {
      hms = 0;
      
    }



  }
}



/* The TimerA ISRs need to be written in assembler because gcc is too brain damaged
   to do the obvious thing when presented with something like A += B (and it also
   insists on pushing a ton of registers on the stack for no reason whatsoever).
*/

__attribute__ ((__naked__,__interrupt__(TIMERA0_VECTOR))) static void
timerA0_isr(void) {
  // These two lines of asm increase the interrupt handling rate by a factor of ~3.
  // At 18MHz, we can hit this interrupt just under 1 million times/sec
  // This line is bad, but we have to do it this way.
  asm("mov r6, %0":"=m" (TACCR0)); //  TACCR0 = softCCR 
  asm("add r5, r6");//  softCCR += timer_rate
  /*
    GCC generates just terrible code (both in speed and *correctness*)
    if we try to write this ISR in C!
  */
  asm("reti");

}


/*
  	The TAIV has the lsb hard-wired to 0 because it is intended to be directly added to
  	the PC in a 3-cycle instruction.  The next instruction executed would be the one that 
  	is located 1, 2, or 5 words ahead of the "add &TAIV, PC" instruction, which is
  	intended to be an unconditional jump (2 cycles) to the real code.  Note that no
  	registers are used (and thus none need be pushed) using this method. Reading TAIV also
  	clears the interrupt flag (which is a Good Thing). 

	timerA1_isr implements the 5-cycle branch in the above paragraph
	TACCR1_isr is the real ISR for the CCR1 interrupt source
	TACCR2_isr is the real ISR for the CCR2 interrupt source
	TAOF_isr is the real ISR for the TA overflow interrupt source

*/
__attribute__ ((__naked__)) static void
TACCR1_isr(void) {
	asm("mov r6, %0":"=m" (TACCR1)); //TACCR1 = softCCR
  	asm("add r5, r6");	//softCCR += ccrRate
	asm("reti");
}


__attribute__ ((__naked__)) static void
TACCR2_isr(void) {
	asm("reti");  // do nothing -- TA2 devices can't get here
}

__attribute__ ((__naked__)) static void
TAOF_isr(void){
	asm("reti"); // do nothing -- TA overflows are expected and normal
}

__attribute__ ((__interrupt__(TIMERA1_VECTOR))) static void
timerA1_isr(void) {
  /*
  TAIV is set according to which source generated the interrupt (CCR1, CCR2, or
  TA overflow).  Adding TAIV to the PC (r0) causes the CPU to branch 1,2, or 5 words
  forward, so the following 5 instructions must do meaningful things in the interrupt
  context (it is best that they are either "reti" or unconditional branches, which are
  both single-word instructions).
  */
  asm("add %0,r0"::"m" (TAIV)); 
  asm("BR %0"::"" (&TACCR1_isr)); // goto CCR1 interrupt handler
  asm("BR %0"::"" (&TACCR2_isr)); // goto CCR2 interrupt handler
  asm("reti"); // reserved flag -- should never be generated
  asm("reti"); // reserved flag -- should never be generated
  asm("BR %0"::"" (&TAOF_isr)); // goto timer overflow handler

}



__attribute__ ((__interrupt__(ADC10_VECTOR))) static void
adc10_isr(void) {
  // Wake up from sleep mode when the ADC finishes a block of data
  __bic_status_register_on_exit(CPUOFF); 
}

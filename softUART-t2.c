#include <msp430.h>
#include <intrinsics.h>
#include "softUART-t2.h"
#define RX_MARK 0x80
#define RX_SPACE 0x00
#define RX_PIN BIT2
#define TX_PIN BIT1

#define RX_PERIOD 3
#define TX_PERIOD 3

unsigned int startTX(unsigned char tx){
/* 
 The argument to this function is the byte that will be sent out the serial port.
 The return value from this function will be put in the tx
 shift register.  
 
 A UART frame begins with a logical zero (space) bit, followed by the lsb of the data byte.
 Bits are sent one by one, in increasing order, and the last data bit is followed by a
 stop bit (logical 1, or "mark").  

 The basic idea here is that Txing consists of setting the output pin equal to the LSB of
 the tx buffer and then right-shifting the word. See the note below (above the return statement)
 for more info about dealing with implementation-dep't shift behavior.  After the frame is
 complete, the tx buffer will contain all 1's.  
 
 The idling state of the UART is the marking state, so it is legal for the UART to
  continue to send 1's after transmission is complete (so the tx side of the UART only
 cares about frame boundaries insofar as it need to avoid startTX()ing the buffer
 before the previous frame has finished sending)
*/
  return ((((unsigned int)tx) << 1)| 0xFE00);
}	

unsigned int doTXbit(unsigned int tx_buf) {
  /*
  Function calls are expensive, so it's better to keep track of the tx timer in the
  calling function (because then the function is only called once per TX_PERIOD).
  In WWVBdsp, keeping track of tx_timer in the function costs about 6% more CPU time in 
  the main loop (measured using oscilloscope on busy bit).
  */

#ifdef SOFTUART_INTERNAL_TX_TIMER 
  static int tx_timer = 0;
  tx_timer++;
  if (TX_PERIOD != tx_timer){
  	return tx_buf;
  }
  tx_timer = 0;
#endif

  // Got this idea from a BYU EE slide.
  //http://ece224web.groups.et.byu.net/lectures/UART.pdf
  if (tx_buf & 0x01){
	  P1OUT |= TX_PIN;
  } else {
    P1OUT &= ~TX_PIN;
  }
  /* since right-shift has implementation-defined behavior for the msb, we need to 
  make sure the msb is always a 1.
  Note that the tx_buf *will* be all ones when the transmission is complete, but that 
  it could have all ones before then also, so checkin for 0xFFFF is not a valid method
   to determine tx complete.
  */
  return (tx_buf>>1)|0x8000;
}

void initTX(void){
  P1REN &= ~TX_PIN;
  P1OUT |= TX_PIN;
  P1DIR |= TX_PIN;
}

void initRX(int *state, unsigned char *rx_buf){
  *state = 0;
  P1DIR &= ~RX_PIN;
  P1SEL &= ~RX_PIN;
  P1REN &= ~RX_PIN;
}

int doRX(int state, unsigned char *rx_buf){
  static int rx_timer = 0;
  /*
	This function implements the UART rx state machine.  
	It takes the previous state, a pointer to a timer and a pointer to the buffer
	as arguments and returns the new state.  The state is passed as a value because 
	it requires slightly fewer cycles.  The timer is kept internally because we need
	to call this function every timer tick anyway (note that making rx_timer static
   precludes running multiple soft UARTs in RX mode, and makes the doRX function
   non-reentrant, the latter of which doesn't really matter here).

    Double ampersand (&&) is the "address of label" operator.
    It is a GCC extension intended to be used with another 
    GCC extension (goto void *).  As a result, this may break
    other compilers.

   We use a jump table here simply because we need a really
   fast way to get to the appropriate state, and GCC is not smart
   enough to compile a switch into one (sadface).  GCC wants to
   do a conditional-branch tree, which is completely facepalm.

  */

  static void * rx_table[] = {
    &&rx_idle,
    &&rx_sync1,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_bit,
    &&rx_stop,
  };

  /*
    This goto compiles into a 7-cycle sequence of instructions on MSP 
   (which could be 6 cycles on MSP if GCC were smarter). 
   
   Aug 2015: We want to avoid assembly as much as practical to keep things portable.
    This is done with an eye towards a potential future port to MSP432, which is 
   an ARM CPU glued to MSP430 peripherals.
    */
 goto *rx_table[state];

 rx_idle:
   if (P1IN & RX_PIN){ /*This will be 1 in the idle state */
     return state;
   } else {
     return state + 1;
   }
 rx_sync1:
    rx_timer = 0;
    return state + 1;
 /*
  Originally this was 
  rx_bit0:
  .
  .
  .
  rx_bit7:
  but the problem was that GCC was dumb and didn't optimize away the fact that all
  these labels referred to the same thing (so after the goto, there was another branch
  to actually get to the code
  */
 
 rx_bit:
  rx_timer +=1;
 /*
  * Note about RX_PERIOD and rx_timer: These values are used to determine when the 
  * state machine shifts to the next state, and the appropriate value for RX_PERIOD depends
  * on how often the main event loop will call doRX.   For example, the WWVB-DSP main
  * loop runs at 30 kHz (= 480 kHz / 8 / 2), so shifting to the next state every three
  * iterations gives a bit period of 1/(10 kHz), which is "pretty close" (4% - hah!) 
  * to 9600 bps.  This works reasonably well in practice (in echo tests), but could 
  * most likely be improved by adding an extra cycle after the 4th or 5th bit.
  */
  if (RX_PERIOD == rx_timer){
    rx_timer = 0;
    state += 1;
    *rx_buf = *rx_buf >> 1;
    if (P1IN & RX_PIN){
      *rx_buf |= BIT7;
    }
  }
  return state;
 rx_stop:
  if(P1IN & RX_PIN) {
    return 0;
  }
  return state;

}




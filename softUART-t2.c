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
 
 The frame begins with a logical zero (space) bit, followed by the lsb of the data byte.
 Bits are sent one by one in increasing order, and the last data bit is followed by a
 stop bit (logical 1, or "mark").  

 The idling state of the UART is the marking state, so it is legal for the UART to
 continue to send 1's after transmission is complete.  

*/
  return ((((unsigned int)tx) << 1)| 0xFE00);
}	

unsigned int doTXbit(unsigned int tx_buf) {
  /*
  Function calls are expensive, so it's better to keep track of the tx timer in the
  calling function (because then the function is only called once per TX_PERIOD).
  In WWVBdsp, keeping track of tx_timer in the function costs about 6% more time in 
  the main loop.
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
  it could have all ones before then also 
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
	to call this function every timer tick anyway.

    Double ampersand (&&) is the "address of label" operator.
    It is a GCC extension intended to be used with another 
    GCC extension (goto void *).  As a result, this may break
    other compilers.

    We use a jump table here simply because we need a really
    fast way to get to the appropriate state, and GCC is not smart
    enough to compile a switch into one.

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
    This goto compiles into a 7-cycle sequence of instructions (which could be 6 if GCC
    were smarter). 
    
    */
 goto *rx_table[state];

 rx_idle:
  if (P1IN & RX_PIN){
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




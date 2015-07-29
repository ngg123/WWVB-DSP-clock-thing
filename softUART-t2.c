#define RX_MARK 0x80
#define RX_SPACE 0x00
#define RX_SYNC_COUNT // clocks to wait to get from mark edge to bit center
#define RX_BIT_COUNT // clocks between bits
#define RX_IDLE 0
#define RX_SYNC 2
#define RX_RUN_0 4
#define RX_RUN_1 6
#define RX_RUN_2 8
#define RX_RUN_3 10
#define RX_RUN_4 12
#define RX_RUN_5 14
#define RX_RUN_6 16
#define RX_RUN_7 18
#define RX_RUN_8 18 // parity or stop bit
#define RX_RUN_9 18 // stop bit
#define RX_STOP (RX_RUN_7 + 2)


unsigned int startTX(unsigned int tx){
/* 
 The return value from this function will be put in the tx
 shift register.  The first bit sent by the UART is a 
 mark ("1"), followed by the lsb.  When the transmission
 is complete, the UART should sent spaces ("0"s), so the
 rest of the shift register should be empty.  

 The advantage of doing it this way is that the calling
 function has complete freedom w.r.t. the number of 
 bits per character and use of parity bits.
*/
	return ((tx & 0xFF) << 1) + 1;

}	

unsigned int doTXbit(unsigned int tx_buf) {
	// The bit-shift operators in C have implementation-defined
	// behavoir.  We want the MSP's "rra" shift, which keeps
	// the value of the msb as the value is shifted, and
	// shifts the lsb into the carry bit of the SR.
	// Got this idea from a BYU EE slide.
	//http://ece224web.groups.et.byu.net/lectures/UART.pdf
	asm("rra %0":"=" tx_buf:);
	if (__get_SR_register(C)) {
		outputMark();
	} else {
		outputSpace();
	}
	return tx_buf;
}


void doRX(int *state, int *timer, unsigned char *rx_buf){
	int rxbit;
	switch(*state) {	
	case RX_IDLE:
		// In the idle state (rx'ing spaces) we just check
		// for the transistion to mark. If present, go to
		// the next state.
		if (getRXbit() == RX_MARK){
			*state = RX_SYNC;
		}
		*timer = 0;
		break;
	case RX_SYNC:	
		*timer++;
		// Wait here for half of the start bit to pass, then
		// set the sample timer to zero and goto next state
		if (*timer == RX_SYNC_COUNT){
			*timer = 0;
			*state = RX_RUN_0;
		}
		break;
	case RX_RUN_0:
	case RX_RUN_1:
	case RX_RUN_2:
	case RX_RUN_3:
	case RX_RUN_4:
	case RX_RUN_5:
	case RX_RUN_6:
	case RX_RUN_7:
		*timer++;
		// When we get to the sample time, measure the input,
		// reset the sample timer, shift the input to the buffer,
		// and goto the next state
		if (*timer == RX_BIT_COUNT){
			*timer == 0;
			*state += 2; //goto next state
			*rx_buf >> 1;
			*rx_buf += getRXbit();
			
		}
		break;
	case RX_STOP:
		*timer++;
		if(*timer == RX_BIT_COUNT){
			if (getRXbit() == RX_MARK) {
				*state = RX_FRAME_ERROR;
			} else {
				*state = RX_IDLE;
			}
		}

		
	case RX_FRAME_ERROR:
		*state = RX_IDLE;
		break;
	}	
	return;

}



unsigned int getRXbit() {
	// return 0x80 or 0x00 depending on input state
	// this is so we can add the return value to the 
	// rx buffer
	
}
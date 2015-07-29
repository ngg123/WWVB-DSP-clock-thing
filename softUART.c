#include <msp430.h>
#include "softUART.h"

void (*enable_start)(void);

void softUART(uartStruct * uart){
  doRXState(uart);
  doTXState(uart);

}

void doRXState(uartStruct * uart){
  //int hat;
  uartStruct s = *uart;
  if (s.rx_space_counter > 8){
    s.rx_state = uart_idle;
    s.rx_space_counter = 0;
  }
  switch(s.rx_state){
  case uart_idle:
    enable_start();
    break;
  case uart_start:
    if (getBit()==uart_space){
      s.rx_state=uart_idle;
      break;
    }
    s.rx_buf = 0;
    s.rx_state++;
    break;
  case uart_bit0: 
  case uart_bit1:
  case uart_bit2:
  case uart_bit3:
  case uart_bit4:
  case uart_bit5:
  case uart_bit6:
  case uart_bit7:
    s.rx_state++;
    s.rx_buf +=s.rx_buf;
    if (getBit() == uart_mark){
      s.rx_buf +=1;
      s.rx_space_counter = 0;
    } else {
      s.rx_space_counter++;
    }
  case uart_stop:
    s.rx_byte = s.rx_buf;
    s.rx_state = uart_idle;
  default:
    s.rx_state = uart_idle;
    break;
  }

}

void doTXState(uartStruct * uart){
  uartStruct s = *uart;
  switch(s.tx_state){
  case uart_idle:
    break;
  case uart_start:
    s.tx_state++;
    s.tx_buf = s.tx_byte;
    setMark();
    break;
  case uart_bit0: 
  case uart_bit1:
  case uart_bit2:
  case uart_bit3:
  case uart_bit4:
  case uart_bit5:
  case uart_bit6:
  case uart_bit7:
    s.tx_state++;
    if (s.tx_buf && 1){
      setMark();
    } else{
      setSpace();
    }
    s.tx_buf >> 1;
    break;  
  case uart_stop:
    setSpace();
    s.tx_state = uart_idle;
    break;
  default:
    s.tx_state = uart_idle;
    break;
  }

}

void setMark(void){

}

void setSpace(void){

}

int getBit(void){
  return 0;
}


void uartStart(uartStruct* uart){
  uart->rx_state=uart_start;
}

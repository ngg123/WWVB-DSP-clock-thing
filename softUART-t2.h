#ifndef SOFTUART_T2_H
#define SOFTUART_T2_H


unsigned int startTX(unsigned char);
unsigned int doTXbit(unsigned int);
void initTX(void);
void initRX(int *, unsigned char *);
int doRX(int, unsigned char *);

#define  SOFTUART_INTERNAL_TX_TIMER

#endif
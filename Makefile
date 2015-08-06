CC=msp430-gcc
CFLAGS=-Os -Wall -g -mmcu=msp430g2231 -Wcpp

OBJS=main.o softUART-t2.o


all: $(OBJS)
	$(CC) $(CFLAGS) -o main.elf $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -fr main.elf $(OBJS)

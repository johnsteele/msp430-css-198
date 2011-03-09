CC=msp430-gcc
#-O2 turns on the C compiler's optimisation, without it code can be quite large and slow. 
#-g Will put necessary info in the compiled binary file to allow symbolic debugging in GDB. 
# it makes binary files rather big, but the actual downloadable code is not affected. 
CFLAGS= -O2 -Os -Wall -g -mmcu=msp430x2012

OBJS=msp430g2xx1_ta_uart9600.o

all: $(OBJS)
	$(CC) $(CFLAGS) -o msp430g2xx1_ta_uart9600.elf $(OBJS)

%.o:%.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -fr msp430g2xx1_ta_uart9600.elf $(OBJS) 


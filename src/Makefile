CC=msp430-gcc
#-O2 turns on the C compiler's optimisation, without it code can be quite large and slow. 
#-g Will put necessary info in the compiled binary file to allow symbolic debugging in GDB. 
# it makes binary files rather big, but the actual downloadable code is not affected. 
CFLAGS= -O2 -Os -Wall -g -mmcu=msp430x2012

OBJS=main.o

all: $(OBJS)
	$(CC) $(CFLAGS) -o main.elf $(OBJS)

%.o:%.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -fr main.elf $(OBJS) 


/**
 * @file	main.c
 * @author	John Steele <EMAIL:programjsteele {at} gmail {dot} com>
 * @version	1.0.0
 * @date
 * 	Created:	Tue 11 Jan 2011 10:52:24 AM PST \n
 * 	Last Update:	Tue 11 Jan 2011 10:52:24 AM PST
 */

/*------------------------------------------------------------------------
 * Description:	This is the main driver for the MSP430G2231 UART project. 
 *
 * UART Description
 * 
 * - The UART function uses hardware features of the timer_A3, and 
 *   software.
 *
 * - The implementation is half-duplex, event-driven, and it supports 8N1
 *   protocal at baud rates from 1200 to 115200, and faster.
 *
 * - The character protocal used is 8N1: 
 * 	 8 data bits, no parity, and one stop bit.
 *
 * - timer_A3 provides automatic start-bit detection, baud-rate generation,
 * 	 and data-bit latching. 
 *
 * - Hardware features of the timer_A3 greatly reduce software and CPU 
 *   overhead typically associated with mcu software UART implementations.
 *
 * - timer_A3 also allows the UART to operate as a background function
 *   simultaneously with other real-time system tasks.
 *
 * - The UART function uses a capture-compare register 0 (CCR0), one of 
 *   the three available with timer_A3. 
 *
 * - CCR0 is used for start-bit detection, baud-rate generation, and data-
 *   bit latching. The other two capture-compare registers are free for 
 *   other tasks. The selection of CCR0 is arbitrary. Any or all CCRx 
 *   registers can be used for the UART function. 
 *
 * - Port pins P1.1 and P2.2 are peripheral-option selections as 
 *   associated with timer_A3 CCR0 (on MSP430x11x(1) derivitives).
 *
 * - P1.1 is selected for transmit, P2.2 for receive.
 *
 * - Peripheral-options are selected for a pin using the peripheral-option
 *   select registers. P1SEL and P2SEL. Because P1.1 is used as an output
 *   this pin mst be configured as an output using the port-1 direction
 *   register (P1DIR). 
 *
 * - P2.2 is required to operate as an input. This is the default for an 
 *   MSP430 port pin. Timer_A3 is configured to run in continuous mode,
 *   allowing timer resources to be available for other functions
 *   simultaneously with the UART. 
 *
 * - CPU register R4 is used for RXTXData - a buffer that shifts in or out
 *   UART data bits. 
 *
 * - CPU register R5 is used for BitCnt, a bit-tracking register. The 
 *   selection of R4 and R5 is arbitrary. Any CPU register or RAM byte
 *   can be used for these functions. 
 *
 * - In receive mode, the capture-compare control register 0 (CCTL0) is
 *   initially configured such that CCR0 captures on the falling edge of
 *   receiving pin P2.2. As the receive line idles high, a falling edge
 *   indicates the beginning of the start-bit. When the UART function is 
 *   ready to receive data, no overhead is put on the CPU even though the 
 *   function is ready to receive a character at any time. CPU resources
 *   are only exercised aftera  start-bit falling edge occurs on P2.2. 
 *   A falling edge on P2.2 captures the current value of the free running
 *   timer_A3 counter register (TAR) in CCR0 independent of any other run-
 *   time activity. Capture is done by timer_A3 hardware, not by software.
 *   
 * - An interrupt is also issued to the CPU. The latency of the interrupt
 *   is not of great concern as the exact time the falling edge triggered
 *   the interrupt is stored in CCR0, independent of other activities. 
 *
 * - After start-bit edge detection, software reconfigures CCTL0 such that
 *   CCR0 is in a compare mode with the first compare to occur in the 
 *   middle of the first data bit. A 1.5 bit offset is added to CCR0, 
 *   positioning the next compare to the middle of the first data bit.
 *
 * - Received data are hardware latched in timer_A3 synchronized capture-
 *   compare input (SCCI). SCCI is a readable latch in CCTL0. In the UART
 *   function, SCCI captures the logic level on the input P2.2 synchronou-
 *   sly with the CCR0 compare. The UART function receives latched data 
 *   from SCCI. Software does not test P2.2 directly. 
 *
 * - RX_Bit bit.w #SCCI,&CCTL0 ; Get bit waiting in SCCI
 *          rrc.b RXTXData     ; Store received bit
 *
 * - After the first data bit, a 1-bit length offset is added to CCR0 to 
 *   position the next capture in the middle of each bit. Eight 
 *   consecutive data bits are latched and received from SCCI by software
 *   into RXTXData bit-by-bit. 
 *
 * - Transmit-mode tasks are simpler because the MSP430 determines when to 
 *   transmit data, and no start-bit edge detect is required. Data stored
 *   in the RXtXData buffer are transmitted on pin P1.1 using timer_A 
 *   CCR0 output hardware. 
 *
 * - CCR0 is preconfigured in compare mode to transmit the next bit using
 *   output mode-control bits in CCTL0. Mode-control bits are pre-
 *   configured to RESET mode (logical 0), or a SET mode (logical 1) to 
 *   occur on the next compare in CR0. When compare occurs, at the 
 *   beginning of a transmitted data bit, CCR0 hardware automatically 
 *   outputs the preconfigured data bit to P1.1  and an interrupt is 
 *   issued. 
 *
 * - With CCR0 hardware automatically outputting data pits on P1.1,
 *   software interrupt latency and bit-timing concerns are reduced. 
 *   It is not necessary for software to prepare CCR0 output latch at an
 *   exact time, only prior to the next bit. 
 *
 * - The UART function software does not directly output data on P1.1, but 
 *   instead preloads the output data bit in CCR0 output latch hardware.  
 *
 *   TX_Bit     rra.b RXTXData            ; LSB is shifted to carry
 *              jc    TX_Mark             ; Jump if bit = 1
 *   TX_Space   bis.w #OUTMOD2,&CCL0      ; TX Space
 *              reti                      ;
 *   TX_Comp    bic.w #CCIE,&CCTL0        ; All Bits RX, disable interrupt 
 *   TX_Mark    bit.w #OUTMOD2,&CCTL0     ; TX Mark
 *              reti                      ;
 *
 * - Before each bit output, software rotates RXTXData to expose the bit
 *   into carry. The appropriate jump is made and CCTL0 output mode is
 *   prepared and a 1-bit length offset is added to CCR0. 
 *
 *
 * Baud Rate Calculation
 *
 * - Timer_A3 CCR0 is used for baud-rate generation. Based on the 
 *   required baud rate, an interval Bitime is calculated. 
 *
 * - Bitime is the lenght in timer_A3 counts between individual bits and
 *   the interval at which timer_A3 latch receives in and transmits out
 *   data bits. 
 *
 * - Bitime is calculated as the timer_A3 clock source divided by the 
 *   baud-rate. 
 *
 * - Timer_A3 has several available clock sources and dividers (see the
 *   (device specific data sheet). Available clock sources for the 
 *   MSP430x11x(1) timer_A3 module included the auxiliary clock
 *   (ACLK), subsystem clock (SMCLK) and two external clocks. 
 *
 * - Example: Consider a 9600-baud-rate with the timer_A3 clock source
 *   selected as the auxiliary ACLK, which is configured to be the same
 *   as a 3.579545-MHz XTAL. 
 *
 *   Bitime = 3 579545 / 9600 = 372.9 ~373
 *   Actual baud-rate = 3 579545 / 373 = 9597
 *
 *   As only an integer value of Bitime can be added to CCR0, the value
 *   373 is used. Assuming 9600-baud and 3.579545-MHz clock source, the
 *   error of rounding Bitime to the nearest integer is less than 0.03% 
 *   per bit.
 *------------------------------------------------------------------------
 * History:	
 * TODO:	Everything!
 *========================================================================
 */


/*===========================================================================*/
/*===============================[ Includes ]================================*/
/*===========================================================================*/
//Take care of by including io.h and setting -mmcu=msp430x2012 
//in cflags. #include	<msp430g2231.h> 
#include	<io.h> /* Input/Output */

/** 
 * @brief P1.1, which is BIT1 and is set to 0x0002 in msp430G2231.h
 * @brief This is the UART Transmit address. 
 */ 
#define	UART_TXD BIT1 

void init_uart (void);

/*===========================================================================*/
/*==================================[ main ]=================================*/
/*===========================================================================*/
int main (void)
{ 
	//WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer. 
	P1DIR |= 0x01;              // Set P1.0 to output direction. 

	return 0;
} /* end main () */


void init_uart (void)
{

} /* end init_uart () */


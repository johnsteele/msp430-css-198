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
 * - The UART function uses hardware features of the timer_A, and 
 *   software.
 *
 * - The implementation is half-duplex, event-driven, and it supports 8N1
 *   protocal at baud rates from 1200 to 115200, and faster.
 *
 * - The character protocal used is 8N1: 
 * 	 8 data bits, no parity, and one stop bit.
 *
 * - timer_A provides automatic start-bit detection, baud-rate generation,
 * 	 and data-bit latching. 
 *
 * - Hardware features of the timer_A greatly reduce software and CPU 
 *   overhead typically associated with mcu software UART implementations.
 *
 * - timer_A also allows the UART to operate as a background function
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
 * - Port pins P1.1 (TXD) and P1.2 (RXD) are peripheral-option selections as 
 *   associated with timer_A CCR0.
 *
 * - P1.1 is selected for transmit, P2.2 for receive.
 *
 * - Peripheral-options are selected for a pin using the peripheral-option
 *   select registers, P1SEL. Because P1.1 is used as an output
 *   this pin must be configured as an output using the port-1 direction
 *   register (P1DIR). 
 *
 * - P1.2 is required to operate as an input. This is the default for an 
 *   MSP430 port pin. Timer_A is configured to run in continuous mode,
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
 *   receiving pin P1.2. As the receive line idles high, a falling edge
 *   indicates the beginning of the start-bit. When the UART function is 
 *   ready to receive data, no overhead is put on the CPU even though the 
 *   function is ready to receive a character at any time. CPU resources
 *   are only exercised after a  start-bit falling edge occurs on P1.2. 
 *   A falling edge on P2.2 captures the current value of the free running
 *   timer_A counter register (TAR) in CCR0 independent of any other run-
 *   time activity. Capture is done by timer_A hardware, not by software.
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
 * - Received data are hardware latched in timer_A synchronized capture-
 *   compare input (SCCI). SCCI is a readable latch in CCTL0. In the UART
 *   function, SCCI captures the logic level on the input P1.2 synchronou-
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
 *   outputs the preconfigured data bit to P1.1 and an interrupt is 
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
 * Baud Rate Calculation
 *
 * - Timer_A CCR0 is used for baud-rate generation. Based on the 
 *   required baud rate, an interval Bitime is calculated. 
 *
 * - Bitime is the length in timer_A counts between individual bits and
 *   the interval at which timer_A latch receives in and transmits out
 *   data bits. 
 *
 * - Bitime is calculated as the timer_A clock source divided by the 
 *   baud-rate. 
 *
 * - Timer_A has several available clock sources and dividers (see the
 *   (device specific data sheet). Available clock sources for the 
 *   MSP430x11x(1) timer_A module included the auxiliary clock
 *   (ACLK), subsystem clock (SMCLK) and two external clocks. 
 *
 * - Example: Consider a 9600-baud-rate with the timer_A clock source
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
 */


/*
 * No need to include msp430g2231.h b/c it's taken care of by in io.h and
 * by setting -mmcu=msp430x2012 in cflags.
 */
#include	<io.h>  	 /* Input/Output */
#include	<signal.h> /* MSPGCC ISR */

/** 
 * @brief UART transmit and receive pin addresses. See msp430G2231.h.
 *       
 */ 
#define	UART_TXD (0x0002) // BIT1 P1.1 (TXD)
#define	UART_RXD (0x0004) // BIT2 P1.2 (RXD)

/**
 * @brief LED1 and LED2 pin addresses. 
 */
#define	LED1 	   (0x0001) // BIT0 P1.0 
#define	LED2 	   (0x0040) // BIT6 P1.6

/**
 * @brief Conditions for 9600 baud software UART, SMCLK = 1MHz
 */
#define	BITIME 13 				// 1,000,000 / 8 / 9600 = ~13
//#define	BIT_COUNT 0xA 	// Number of bits, 8 data + ST / SP.  
#define	BIT_COUNT 0xB 	  // see msp430g2xx1_ta_uart9600.c

 /**
 * @brief Initialize LED1 and LED2. 
 */
void init_LEDs   (void);

/**
 * @brief Initialize TimerA for capture/compare interrupts.
 */
void init_timerA (void);

/**
 * @brief Initialize Port 1 for UART TXD and RXD.
 */
void init_uart   (void);

/**
 * @brief Initialize the clock frequency. 
 */
void init_clock (void);

/**
 * @brief Transmits one  using UART. 
 */
void TimerA_UART_tx_byte (int number);

/**
 * @brief Number of transmitted bits. 
 */
unsigned int tx_bit_count = 0;

/**
 * @brief Number of receieved bits. 
 */
unsigned int rx_bit_count = 0;

/**
 * @brief Value to be transmitted.
 */
unsigned int tx_data;

/**
 * @brief Value received. 
 */
unsigned int rx_data;

int main (void)
{ 
  /*
	 * Stop watchdog timer.  
	 * According to the datasheet the watchdog timer starts
	 * automatically after powerup. It must be configured or
	 * halted at the beginning of code execution to avoid a 
	 * system reset. Furthermore, the watchdog timer register
	 * (WDTCTL) is password protected, and requires the upper 
	 * byte durring write operations to be 0x5A, which is the
	 * value associated with WDTPW.
	 */
	WDTCTL = WDTPW + WDTHOLD;

 	// Initialize LEDs, Clock, and Timer. 
	init_LEDs   ();
	init_clock  ();
	init_timerA ();
	init_uart   ();

	//Enable global interrupts, specific to the mspgcc compiler. 
	eint ();

	unsigned int tx_value = 13;
	while (1) { 
		if (tx_value == 100) tx_value = 0;

		TimerA_UART_tx_byte (tx_value);	

		tx_value++;	
	} /* end while () */

	return 0;
} /* end main () */


//=============================================================================
/**
 * @brief Initialize LED1 (P1.0) and LED2 (P1.6).
 */
//=============================================================================
void init_LEDs (void)
{ 
	// Set for output.
	P1DIR |= (BIT0 + BIT6); 

	// Initialize all IO.
	P1OUT = 0x00;
} /* end init_LED1 */


//=============================================================================
/**
 * @brief Initialize TimerA for capture/compare interrupts.
 */ 
//=============================================================================
void init_timerA (void)
{ 
	TACCTL0 = OUT;

	// Sync, Negative Edge, Capture, Interrupt. 
	TACCTL1 = SCS + CM1; // + CCIE; 

	// Set TimerA to submain clock, and run on continuous mode.  
	// Continuous means count up to FFFF, rolls over to 0000, back up to FFFF, etc.
	TACTL = TASSEL_2 + MC_2;
} /* end init_timerA */



//=============================================================================
/**
 * @brief Initialize the clock frequency. 
 */
//=============================================================================
void init_clock (void)
{ 
	DCOCTL = 0x00;

	// Set clock to 1mHz
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL  = CALDCO_1MHZ; 
} /* end init_clock */



//=============================================================================
/**
 * @brief Initialize P1 for UART TXD and RXD. 
 */
//=============================================================================
void init_uart (void)
{
	// Select TXD and RXD pins. 
	P1SEL |= UART_TXD + UART_RXD; 

	// Set P1.1 (TXD) to output direction.  
	P1DIR |= UART_TXD; 
} /* end init_uart () */



//=============================================================================
/**
 * @brief Transmit ISR for TimerA.
 */
//=============================================================================
interrupt (TIMERA0_VECTOR) TA0_IntServiceRoutine (void)
{ 
	// If all bits have been trasmitted, disable interrupts. 
	if (tx_bit_count == 0)  
		TACCTL0 &= ~CCIE;	

	else {

		// Turn green LED on to signal sending a bit.
		P1OUT ^= BIT6; 
	
		// Add offset to CCRx
		TACCR0 += BITIME; 
		
		if (tx_data & 0x01) 
			TACCTL0 &= ~OUTMOD2; 	// TX Mark '1'

		else 
			TACCTL0 |= OUTMOD2;  	// TX Space '0'

		// Move to next bit to transmit.
		tx_data >>= 1;

		// Decrement total number of bits to send. 
		tx_bit_count--;

	  // Turn green LED off to signal finished sending a bit.
		P1OUT ^= BIT6; 
	} // end else 
} /* end IntServiceRoutine () */



//=============================================================================
/**
 * @brief Transmits one byte using UART. 
 */
//=============================================================================
void TimerA_UART_tx_byte (int the_tx_data)
{
	// Ensure last byte got transmitted.
	while (TACCTL0 & CCIE); // Spin wheels until interrupts are disabled.

	// Set number of bits to send.  
	tx_bit_count = BIT_COUNT;	

	// Toggle red LED to signal sending a new byte.
	P1OUT ^= BIT0; 

	// Current state of TA counter. 
	TACCR0 = TAR;	

	// Set TXD on EQU0, Interrupt
	TACCTL0 = OUTMOD0 + CCIE;

	// Load global value to transmit. 
	tx_data = the_tx_data;

	// Add stop bit to the tx_data
	tx_data |= 0x100;

	// Add a space for the start bit.
	tx_data <<= 1;

	// Time until first bit to transmit.
	TACCR0 += BITIME;

	// Toggle red LED to signal finished sending byte.
	P1OUT ^= BIT0; 
} /* end TimerA_UART_tx () */


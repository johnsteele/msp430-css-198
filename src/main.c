//========================================================================
/**
 * @file	/home/johnsteele/Desktop/css198/msp430-css-198/src/main.c
 * @author	John Steele <EMAIL:programjsteele {at} gmail {dot} com>
 * @version	1.0.0
 * @date
 * 	Created:	Tue 11 Jan 2011 10:52:24 AM PST \n
 * 	Last Update:	Tue 11 Jan 2011 10:52:24 AM PST
 */
/*------------------------------------------------------------------------
 * Description:	This is the main driver for the MSP430G2231 project.
 * 
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





// Toggles the LED. 
void run_led (void);


/*===========================================================================*/
/*==================================[ main ]=================================*/
/*===========================================================================*/
int main (void)
{

	WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer. 
	P1DIR |= 0x01;             // Set P1.0 to output direction. 

	while (1) {
		int i;
		P1OUT ^= 0x01; // Toggle P1.0 using exclusive or. 
		for (i = 0;i < 5000;i++);
	}

	return 0;
} /* end main () */

void run_led (void)
{


} /* end run_led () */

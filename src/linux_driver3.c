//========================================================================
/**
 * @file	uart_driver.c
 * @author	John Steele <EMAIL:programjsteele {at} gmail {dot} com>
 * @version	1.0.0
 * @date January 12, 2011
 *
 * Description:	This driver sends and receives data to the MSP430G2231
 * 				 		  MCU using UART.  
 */
//========================================================================

#include	<termios.h> 	/* open/close */
#include	<fcntl.h>			/* O_* flags */
#include	<stdio.h> 		/* printf */ 
#include	<unistd.h>    /* STDIN_FILENO */

static const char uart_device_name [] = "../../../../../../../dev/launchpad";

/**
 * @brief Used to remember original terminal attributes.
 */
struct termios saved_attributes;
void set_input_mode (void);
int main (void)
{
	/*
	 * O_RDWR   - Port reading and writing.
	 * O_NOCTTY - Port never becomes the controlling terminal of the process.
	 * O_NDELAY - Use non-blocking I/O. On some systems this also means RS232
	 * 					  DCD signal line is ignored.  
	 */
	int fd = open (uart_device_name, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd == -1)  {
		printf ("%s %s \n", "Failed to open port - ", uart_device_name);
		return 1;
	}
	else 
		printf ("%s %s \n", "Device Open - ", uart_device_name);

	if (!isatty(fd))  {
		printf ("%s %s \n", "Device not a serial (tty) device - ", uart_device_name);	
		return 1;
	}
	
	int c;

	while (c != 100) {
	
		// If new data is available on the serial port, print it out.	
		if (read (fd, &c, 1) > 0) {
			printf ("data: %i \n", c);
		}
		

//		else printf ("No data available...\n");
			//write (STDOUT_FILENO, &c, 1);

		// If new data is available on the console, send it to the serial port.
		//if (read (STDIN_FILENO, &c, 1) > 0) write (fd, &c, 1); 
		//else printf ("No data to send...\n");


	} // end while 

	close (fd);	
	return 0;
} /* end main () */


void set_input_mode (void)
{


} /* end set_input_mode () */

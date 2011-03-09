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

static const char uart_device_name [] = "../../../../../../../dev/ttyACM0";

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

	if (fd == -1) 
		printf ("%s %s \n", "Failed to open port - ", uart_device_name);

	else 
		printf ("%s %s \n", "Device Open - ", uart_device_name);

	if (!isatty(fd)) 
		printf ("%s %s \n", "Device not a serial (tty) device - ", uart_device_name);	


	/*
	 * Input flags - Turn off input processing. 
	 * convert break into null byte, no CR to NL translation,
	 * no input parity check, don't strip high bit off,
	 * no XON/XOFF software flow control.
	 */

  struct termios config;
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
											INLCR  | PARMRK | INPCK | ISTRIP | IXON);


	//	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | 
	//											ONOCR | ONOEOT | OFILL | OLCUC | OPOST);
	config.c_oflag = 0;


	/*
	 * No line processing:
	 * echo off, echo newline off, canical mode off,
	 * extended input processing off, signal chars off
	 */
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	//------------------------------------------------
	/* Set character size
	 *
	 * Turn off character processing
	 * clear current char size mask, no parity checking,
	 * no output processing, force 8 bit input.
	 */
	config.c_cflag &= ~(CSIZE | PARENB); // Mask the character size bits. 
	config.c_cflag |= CS8; // Select 8 data bits.



	//------------------------------------------------
	/*
	 * One input byte is enough to return from read ()
	 * Inter-character timer off.
	 */
	config.c_cc [VMIN]  = 1;
	config.c_cc [VTIME] = 0;

	/*
	 * Communication speed (simple version, using the predefined
	 * constants).
	 */
	if (cfsetispeed (&config, B9600) < 0 || cfsetospeed (&config, B9600) < 0) {
		// error handling recommended.
		printf ("Unable to set input speed to B9600.\n");
		return -1;
	}

	/*
	 * Finally apply the configuration.
	 */
	if (tcsetattr (fd, TCSAFLUSH, &config) < 0) {
		// error handling recommended.
		printf ("Unable to set configuration.\n");
		return -1;
	}

//	unsigned char c = 'D';
	unsigned char c = 'D';

	while (c != 101) {
	
		// If new data is available on the serial port, print it out.	
		if (read (fd, &c, 1) > 0) {
			printf ("%s %c \n", "Data: ", c);
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

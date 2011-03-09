//========================================================================
/**@file	/home/johnsteele/Desktop/css198/msp430-css-198/src/linux_driver2.c
 * @author	John Steele <EMAIL:programjsteele {at} gmail {dot} com>
 * @version	«version»
 * @date
 * 	Created:	Mon 07 Mar 2011 07:04:48 PM PST \n
 * 	Last Update:	Mon 07 Mar 2011 07:04:48 PM PST
 */
/*------------------------------------------------------------------------
 * Description:	
 * 
 *------------------------------------------------------------------------
 * History:	
 * TODO:	
 *========================================================================
 */


/*===========================================================================*/
/*===============================[ Includes ]================================*/
/*===========================================================================*/



/*===========================================================================*/
/*==================================[  ]===================================*/
/*===========================================================================*/
#include	<unistd.h>
#include	<stdio.h>
#include	<termios.h>
#include	<stdlib.h>

/**
 * @brief Used to remember original terminal attributes. 
 */
struct termios saved_attributes;

void reset_input_mode (void);
void set_input_mode (void);


int main (void) 
{

		char c;
		set_input_mode ();

		while (1) {
			read (STDIN_FILENO, &c, 1);

			if (c == '\004') break; // C-d */
		
			else putchar (c); 
		}
	return EXIT_SUCCESS; 
}

void reset_input_mode (void)
{

}


void set_input_mode (void)
{
	struct termios tattr;
	
	/* Make sure stdin is a terminal */
	if (!isatty (STDIN_FILENO)) {
		fprintf (stderr, "Not a terminal.\n");
		exit (EXIT_FAILURE);
	}

	/* Save the terminal attributes so we can restore them later. */
	tcgetattr (STDIN_FILENO, &saved_attributes);
	atexit (reset_input_mode);

	/* Set the funny terminal modes. */
	tcgetattr (STDIN_FILENO, &tattr);
	tattr.c_lflag &= ~(ICANON|ECHO); // Clear ICANON and ECHO.
	tattr.c_cc [VMIN] = 1;
	tattr.c_cc [VTIME] = 0;
	tcsetattr (STDIN_FILENO, TCSAFLUSH, &tattr);
}


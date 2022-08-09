/*
 * HPGL2CBM1520_plot
 * 
 * This project plots a HPGL file to a Commodore 1520 plotter.
 */

//#include <conio.h>
#include <stdio.h>

void main(void)
{
    char filename[20];

    cputs("Hello world!\r\n");

    cputs("File to plot:");
    gets(filename, sizeof filename);
    cputs("Plotting file ");
    cputs(filename)
    
}

void read_HPGL_command()

void gets(char *s, unsigned char size)
{
    unsigned char c;
    
    while strlen(s) < size
    {
        c = cgetc();
    }
}
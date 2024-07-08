/*
 * CFile1.c
 *
 * Created: 09/06/2024 15:40:00
 *  Author: mast274931
 */

#define	F_CPU		3686400

#include "lcd.h"
#include <util/delay.h>
#include <avr/io.h>

/* section with custom symbols */
/* degree symbol */
uint8_t deg_sym[8] = {
	0b01110,
	0b01010,
	0b01110,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};
/* turn signal left off symbol*/
uint8_t ts_off_l_sym[8] = {	
	0b00010,
	0b00101,
	0b01001,
	0b10001,
	0b01001,
	0b00101,
	0b00010,
	0b00000
};
/* turn signal right off symbol*/
uint8_t ts_off_r_sym[8] = {
	0b01000,
	0b10100,
	0b10010,
	0b10001,
	0b10010,
	0b10100,
	0b01000,
	0b00000,
};
/* turn signal left on symbol*/
uint8_t ts_on_l_sym[8] = {
	0b00010,
	0b00111,
	0b01111,
	0b11111,
	0b01111,
	0b00111,
	0b00010,
	0b00000
};
/* low beam off symbol*/
uint8_t lb_off_sym[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11111,
	0b01110,
	0b01110
};
/* low beam on symbol*/
uint8_t lb_on_sym[8] = {
	0b00000,
	0b10101,
	0b10101,
	0b10101,
	0b00000,
	0b11111,
	0b01110,
	0b01110
};
/* cornering light off symbol*/
uint8_t cl_off_sym[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11111,
	0b01110,
	0b01110
};
/* cornering light on symbol*/
uint8_t cl_on_sym[8] = {
	0b01001,
	0b10010,
	0b10010,
	0b01001,
	0b00000,
	0b11111,
	0b01110,
	0b01110
};


void uint8t2ascii(uint8_t num, char *text)
{
	text[0] = 0x30;		// base of ascii numbers
	text[1] = 0x30;		// base of ascii numbers
	text[2] = 0x30;		// base of ascii numbers
	text[3] = 0x00;		// null terminator

	/* Convert the number to ascii */
	text[0] += num / 100;
	num = num % 100;
	text[1] += num / 10;
	num = num % 10;
	text[2] += num;
}

void int8t2ascii(int8_t num, char *text)
{
	uint8_t is_negative = 0;

	/* Handle negative numbers */
	if (num < 0) {
		is_negative = 1;
		num = -num;
	}
	
	text[0] = is_negative ? '-' : '+';		// decide which sign to use
	text[1] = 0x30;							// base of ascii numbers
	text[2] = 0x30;							// base of ascii numbers
	text[3] = 0x30;							// base of ascii numbers
	text[4] = 0x00;							// null terminator
	
	/* Convert the number to ascii */
	text[1] += num / 100;
	num = num % 100;
	text[2] += num / 10;
	num = num % 10;
	text[3] += num;
}

void int16t2ascii(int16_t num, char *text)
{
	uint8_t is_negative = 0;

	/* Handle negative numbers */
	if (num < 0) {
		is_negative = 1;
		num = -num;
	}
		
	text[0] = is_negative ? '-' : '+';		// decide which sign to use
	text[1] = 0x30;							// base of ascii numbers
	text[2] = 0x30;							// base of ascii numbers
	text[3] = 0x30;							// base of ascii numbers
	text[4] = 0x30;							// base of ascii numbers
	text[5] = 0x30;							// base of ascii numbers
	text[6] = 0x00;							// null terminator
		
	/* Convert the number to ascii */
	text[1] += num / 10000;
	num = num % 10000;
	text[2] += num / 1000;
	num = num % 1000;
	text[3] += num / 100;
	num = num % 100;
	text[4] += num / 10;
	num = num % 10;
	text[5] += num;
}

void lcd_write(char data)
{
	PORTD &= ~(0b11111100);	// clear bits 2 to 7 of port d
	PORTD |= RS;			// rising edge of E
	PORTD |= E;
	PORTD |= data & UHB;	// actual soft reset command
	PORTD &= ~E;			// falling edge of E
		
	PORTD |= E;				// rising edge of E
	PORTD &= ~UHB;			// clear UHB
	PORTD |= data << 4;		// actual soft reset command
	PORTD &= ~E;			// falling edge of E
	_delay_ms(1);
}

void lcd_text(char *text)
{
	uint8_t j = 0;
	
	while (text[j] != 0)
	{
		lcd_write(text[j]);
		j++;					// increment j
	}
}

void lcd_cmd(uint8_t cmd)
{
	PORTD &= ~(0b11111100);	// clear bits 2...7 of PORTD
	PORTD |= E;				// rising edge of E
	PORTD |= cmd & UHB;		// set bits 4...7 of PORTD according to UHB of cmd
	PORTD &= ~E;			// falling edge of E
	
	PORTD |= E;				// rising edge of E
	PORTD &= ~UHB;			// clear bits 4...7 of PORTD
	PORTD |= cmd << 4;		// set bits 4...7 of PORTD according to LHB of cmd
	PORTD &= ~E;			// falling edge of E
	_delay_ms(1);			// wait 2ms
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
	uint8_t cmd = 0x00;
	
	if (row == 0) {
		cmd |= 0x80;
	}
	else if (row == 1) {
		cmd |= 0xC0;
	}
	
	if (column < 16) {
		cmd |= column;
	}
	
	lcd_cmd(cmd);
}

void lcd_clear() {
	lcd_cmd(0x01);
}

void lcd_create_custom_char(uint8_t location, uint8_t charmap[]) {
	location &= 0x07; // Only 8 locations (0-7) are available
	lcd_cmd(0x40 | (location << 3)); // Set CGRAM address

	for (int i = 0; i < 8; i++) {
		lcd_write(charmap[i]); // Write character data
	}
}

void lcd_init(void)
{
	_delay_ms(15);			// wait 15ms
	PORTD &= ~(0b11111100);	// clear bits 2...7 of PORTD
	PORTD |= E;				// rising edge of E
	PORTD |= RESET;			// set soft reset command
	PORTD &= ~E;			// falling edge of E
	_delay_ms(5);			// wait 5ms
	
	PORTD |= E;				// rising edge of E
	PORTD |= RESET;			// set soft reset command
	PORTD &= ~E;			// falling edge of E
	_delay_ms(2);			// wait 2ms
	
	PORTD |= E;				// rising edge of E
	PORTD |= RESET;			// set soft reset command
	PORTD &= ~E;			// falling edge of E
	_delay_ms(1);			// wait 2ms
	
	PORTD &= ~(0b11111100);	// clear bits 2...7 of PORTD
	PORTD |= E;				// rising edge of E
	PORTD |= (1<<PD5);		// set 4 bit interface
	PORTD &= ~E;			// falling edge of E
	_delay_ms(1);			// wait 2ms
	
	lcd_cmd(0x28);			// set 4 bit interface, 2 lines 5x7 dots
	lcd_cmd(0x08);			// LCD off
	lcd_cmd(0x01);			// clear LCD
	lcd_cmd(0x06);			// ENTRY MODE: no shift, increase
	lcd_cmd(0x0F);			// LCD on
	
	_delay_ms(2);			// wait 2ms

	/* create custom characters */
	lcd_create_custom_char(DEG_SYM, deg_sym);
	lcd_create_custom_char(TS_OFF_L_SYM, ts_off_l_sym);
	lcd_create_custom_char(TS_OFF_R_SYM, ts_off_r_sym);
	lcd_create_custom_char(TS_ON_L_SYM, ts_on_l_sym);
	lcd_create_custom_char(LB_OFF_SYM, lb_off_sym);
	lcd_create_custom_char(LB_ON_SYM, lb_on_sym);
	lcd_create_custom_char(CL_OFF_SYM, cl_off_sym);
	lcd_create_custom_char(CL_ON_SYM, cl_on_sym);
}

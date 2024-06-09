/*
 * CFile1.c
 *
 * Created: 09/06/2024 15:40:00
 *  Author: mast274931
 */

/* F_CPU needs to be defined before delay is imported */
#define	F_CPU		3686400

#include <util/delay.h>
#include <avr/io.h>

#define RS			0b00000100
#define E			0b00001000
#define RESET		0b00110000
#define UHB			0b11110000
#define LHB			0b00001111

void int2ascii(uint8_t num, char *text)
{
	text[0] = 0x30;		// base of ascii numbers
	text[1] = 0x30;		// base of ascii numbers
	text[2] = 0x30;		// base of ascii numbers
	text[3] = 0x00;
	
	text[0] += num / 100;
	num = num % 100;
	text[1] += num / 10;
	num = num % 10;
	text[2] += num;
}

void lcd_text(char *text)
{
	uint8_t j = 0;
	
	while (text[j] != 0)
	{
		PORTD &= ~(0b11111100);	// clear bits 2 to 7 of port d
		PORTD |= RS;			// rising edge of E
		PORTD |= E;
		PORTD |= text[j] & UHB;	// actual soft reset command
		PORTD &= ~E;			// falling edge of E
		
		PORTD |= E;				// rising edge of E
		PORTD &= ~UHB;			// clear UHB
		PORTD |= text[j] << 4;		// actual soft reset command
		PORTD &= ~E;			// falling edge of E
		_delay_ms(1);
		
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
}

/*
 * IncFile1.h
 *
 * Created: 09/06/2024 15:38:37
 *  Author: mast274931
 */ 

#include <avr/io.h>

#ifndef LCD_H_
#define LCD_H_

#define DEG_SYM			0x00
#define TS_OFF_L_SYM	0x01
#define TS_OFF_R_SYM	0x02
#define TS_ON_L_SYM		0x03
#define LB_OFF_SYM		0x04
#define LB_ON_SYM		0x05
#define CL_OFF_SYM		0x06
#define CL_ON_SYM		0x07

#define RS			0b00000100
#define E			0b00001000
#define RESET		0b00110000
#define UHB			0b11110000
#define LHB			0b00001111

void uint8t2ascii(uint8_t num, char *text);
void int8t2ascii(int8_t num, char *text);
void int16t2ascii(int16_t num, char *text);
void lcd_text(char *text);
void lcd_cmd(uint8_t cmd);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear();
void lcd_init(void);
void lcd_write(char data);
void lcd_create_custom_char(uint8_t location, uint8_t charmap[]);

#endif /* LCD */
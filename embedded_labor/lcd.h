/*
 * IncFile1.h
 *
 * Created: 09/06/2024 15:38:37
 *  Author: mast274931
 */ 

#include <avr/io.h>

#ifndef LCD_H_
#define LCD_H_

void int2ascii(uint8_t num, char *text);
void lcd_text(char *text);
void lcd_cmd(uint8_t cmd);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear();
void lcd_init(void);

#endif /* LCD */
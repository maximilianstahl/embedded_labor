/*
 * main.c
 *
 * Created: 06/06/2024 09:52:00
 * Author : mast274931
 */ 

/* F_CPU needs to be defined before delay is imported */
#define	F_CPU		3686400

#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRUE		0x01
#define FALSE		0x00

#define DB_TIME_MS	50	

/* defines for button status register */
#define TS_BTN		1
#define LB_BTN		2

/* defines for lcd status register */
#define TURN_SIG	1
#define LOW_BEAM	2
#define	CORNERING	3	
#define STEERING	4

/* defines for control register*/
#define STEER_DONE	1
#define TURN_OFF_TS	2

typedef enum {
	BTN_RELEASED = 1,	// released state
	BTN_PRESSED = 2,	// pressed state
	BTN_PRESSED_DB = 3,	// pressed debounce state
	BTN_RELEASED_DB = 4	// released debounce state
} ButtonState;

typedef enum {
	TS_IDLE = 1,	// idle state
	TS_COMF = 2,	// comfort mode
	TS_CONT = 3		// continuous mode
} TurnSigState;

#define IS_SET(reg, bit)		((reg) & (1 << (bit)))
#define SET_BIT(reg, bit)		((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)		((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)	((reg) ^= (1 << (bit)))

volatile uint8_t btn_stat_reg = 0x00;	// register for button statuses
volatile uint8_t lcd_stat_reg = 0x00;	// register to tell LCD what to display
volatile uint8_t ctrl_reg = 0x00;		// register for general control exchange

void timer_intr_init(void)
{
	/* Timer0 settings */
	TCCR0 = (1 << CS01);	// prescaler 8
	
	/* Timer1 settings */
    TCCR1A = 0x00;				// Set Timer1 to CTC mode (WGM12), 
    TCCR1B = (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10);	// prescaler 64 (for 1 ms interrupt)
    /* Set the compare values for OCR1A and OCR1B */
    OCR1A = 56;		// 3,686,400 / ?64 * 1,000 - 1 = 57,6 ~= 56
    OCR1B = 56;		// 3,686,400 / ?64 * 1,000 - 1 = 57,6 ~= 56
	
	/* Timer2 settings */
	TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21);		// enable fast pwm, with clear oc2 at compare match, prescaler 8
	
	TIMSK |= (1 << TOIE2) | (1 << OCIE1B) | (1 << OCIE1A) | (1 << TOIE0);	// overflow interrupt
	sei();													// global interrupt enable
}

void adc_init(void) 
{
	ADMUX |= (1 << REFS0) | (1 << ADLAR);	// AVcc as reference, left adjust -> 8 bit result
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADFR);	// prescaler to 64, free running, enable adc, start conversion
	
	ADCSRA |= (1 << ADSC);	// start conversion
}

volatile TurnSigState turn_sig_state = TS_IDLE;				// initial state is always idle
volatile ButtonState ts_btn_state = BTN_RELEASED;		// initial state is always idle

int main(void)
{
	uint8_t AD_val = 0;				// define var ad value
	char lcd_str[17];
	
	DDRD |= 0xFC;					// set bits 2...7 as outputs
	DDRB |= (1 << PB4);				// PB4 as output
	DDRC &= ~(1 << PC0);			// PC0 as input
	PORTC |= (1 << PC2);			// PC2 as input
	
	timer_intr_init();				// init timer counter interrupts
	lcd_init();						// init LCD
	adc_init();						// init analog digital converter
	
	while (TRUE)
	{
		if ((lcd_stat_reg & (1 << TURN_SIG))) {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("ON ");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("OFF");				// put value to LCD
		}
		
		if ((lcd_stat_reg & (1 << STEERING))) {
			lcd_set_cursor(0, 4);			// write to the fourth column of the first row
			lcd_text("TRUE ");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 4);			// write to the fourth column of the first row
			lcd_text("FALSE");				// put value to LCD
		}
		
		AD_val = ADCH;
		int2ascii(AD_val, lcd_str);			// convert int to ascii representation
		lcd_set_cursor(1, 0);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		
		int2ascii(turn_sig_state, lcd_str);	// convert int to ascii representation
		lcd_set_cursor(1, 4);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to lcd
		
		int2ascii(ts_btn_state, lcd_str);	// convert int to ascii representation
		lcd_set_cursor(1, 8);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to lcd
		
		_delay_ms(100);
	}
}

ISR (TIMER0_OVF_vect)
{
	static uint8_t steering_initiated = FALSE;	// var to tell if steering process is initiated

	/* steering processing */
	if (!steering_initiated) {
		if (!(((127 - 42) < ADCH) && (ADCH < (127 + 42)))) {
			/* steering process is initiated -> not -10° < lw < 10° */
			steering_initiated = TRUE;
			lcd_stat_reg &= ~(1 << STEERING);	// unset steering done
				
		}
		else {
			ctrl_reg &= ~(1 << TURN_OFF_TS);	// unset turn off turn signal
		}
	}
	else {
		if (((127 - 42) < ADCH) && (ADCH < (127 + 42))) {
			/* steering done -> -10° < lw < 10° */
			steering_initiated = FALSE;
			lcd_stat_reg |= (1 << STEERING);	// set steering done
			ctrl_reg |= (1 << TURN_OFF_TS);		// set turn off turn signal
		}
	}
}

ISR (TIMER1_COMPA_vect)
{
	/* ISR primarily used for debouncing */
	//static ButtonState ts_btn_state = IDLE_BTN;		// initial state is always idle
	static uint16_t ctr = 0;
	
	/* turn signal state machine */
	switch (ts_btn_state) {
		case BTN_RELEASED:
			if (!(PINC & (1 << PC2))) {
				/* if button is pressed, start debouncing */
				ts_btn_state = BTN_PRESSED_DB;
			}
			break;
		case BTN_PRESSED_DB:
			/* debouncing */
			if (ctr > DB_TIME_MS) {
				if (!(PINC & (1 << PC2))) {
					/* debounce successful -> set button state and go to pressed state*/
					btn_stat_reg |= (1 << TS_BTN);
					ts_btn_state = BTN_PRESSED;
				}
				else {
					/* debounce unsuccessful -> back to not pressed */
					ts_btn_state = BTN_RELEASED;
				}
				/* reset db counter */
				ctr = 0;
			}
			else {
				ctr += 1;
			}
			break;
		case BTN_PRESSED:
			/* check if button is released*/
			if (PINC & (1 << PC2)) {
				/* if so, reset button state and go to released */
				ts_btn_state = BTN_RELEASED_DB;
			}
			break;
		case BTN_RELEASED_DB:
			/* button release debounce (prevents strange behavior)*/
			if (ctr > DB_TIME_MS) {
				if (PINC & (1 << PC2)) {
					/* debounce successful -> set button state and go to pressed state*/
					btn_stat_reg &= ~(1 << TS_BTN);
					ts_btn_state = BTN_RELEASED;
				}
				else {
					/* debounce unsuccessful -> back to pressed */
					ts_btn_state = BTN_PRESSED;
				}
				/* reset db counter */
				ctr = 0;
			}
			else {
				ctr += 1;
			}
		default:
			break;
	}
}

ISR (TIMER1_COMPB_vect)
{
	// static State turn_sig_state = IDLE;		// initial state is always idle
	static uint16_t ctr = 0, comf_ctr = 0, desc_ctr = 0;
	static uint8_t go_to_cont = FALSE;
	static uint8_t exit_cont = FALSE;
	
	/* turn signal state machine */
	switch (turn_sig_state) {
		case TS_IDLE:
			if (btn_stat_reg & (1 << TS_BTN)) {
				/* if button is pressed and valid, go to comfort mode */
				turn_sig_state = TS_COMF;
				
				/* initially turn on LED */
				PORTB |= (1 << PB4);
				lcd_stat_reg |= (1 << TURN_SIG);
			}
			break;
		case TS_COMF:
			/* only allow check during the first second */
			if ((comf_ctr < (1000 - DB_TIME_MS)) && (btn_stat_reg & (1 << TS_BTN))) {
				desc_ctr += 1;
				
				/* check if button is held for 1 second minus debounce time */
				if (desc_ctr >= (1000 - DB_TIME_MS)) {
					go_to_cont = TRUE;
				}
			}
			
			/* 5 because initial state is on, so we just need 5 instead of 6 led toggles */
			if (comf_ctr < 5) {
				if (ctr > 500) {
					comf_ctr += 1;
					ctr = 0;
					
					PORTB ^= (1 << PB4);				// toggle led
					lcd_stat_reg ^= (1 << TURN_SIG);	// toggle status register
				}
				else {
					ctr += 1;
				}
			}
			else {
				/* check if we go to continuous or if we stop */
				if (go_to_cont) {
					turn_sig_state = TS_CONT;
					go_to_cont = FALSE;
				}
				else {
					turn_sig_state = TS_IDLE;
				}
				
				/* comfort mode done -> reset all counters */
				desc_ctr = 0;
				comf_ctr = 0;
				ctr = 0;
			}
			break;
		case TS_CONT:
			if (ctr > 500) {
				PORTB ^= (1 << PB4);				// toggle led
				lcd_stat_reg ^= (1 << TURN_SIG);	// toggle status register
				ctr = 0;
			}
			else {
				ctr += 1;
			}
			
			/* check if either button is pressed again or steering is done*/
			if ((btn_stat_reg & (1 << TS_BTN)) || ctrl_reg & (1 << TURN_OFF_TS)) {
				exit_cont = TRUE;
			}
			
			/* button condition is required so loop does not instantly start again */
			if (exit_cont && !(btn_stat_reg & (1 << TS_BTN)))
			{
				ctrl_reg &= ~(1 << TURN_OFF_TS);	// unset turn off turn signal
				
				ctr = 0;
				turn_sig_state = TS_IDLE;
				exit_cont = FALSE;
				/* manually turn off led */
				PORTB &= ~(1 << PB4);
				lcd_stat_reg &= ~(1 << TURN_SIG);
			}
			
			break;
		default:
			break;
	}
}

ISR (TIMER2_OVF_vect)
{
	
}
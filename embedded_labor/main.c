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

typedef enum {
	LB_IDLE = 1,	// idle state
	LB_DIM = 2,		// dimming mode
	LB_ON = 3		// on mode
} LowBeamState;

#define IS_SET(reg, bit)		((reg) & (1 << (bit)))
#define SET_BIT(reg, bit)		((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)		((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)	((reg) ^= (1 << (bit)))

volatile uint8_t btn_stat_reg = 0x00;	// register for button statuses
volatile uint8_t lcd_stat_reg = 0x00;	// register to tell LCD what to display
volatile uint8_t ctrl_reg = 0x00;		// register for general control exchange

volatile uint8_t dim_val = 0xFF;

void turn_signal_processing(void);
void low_beam_processing(void);
void steering_processing(void);
void turn_signal_button_debounce(void);
void low_beam_button_debounce(void);
void timer_intr_init(void);
void adc_init(void);


int main(void)
{
	uint8_t AD_val = 0;				// define var ad value
	char lcd_str[17];
	
	DDRD |= 0xFC;					// set bits 2...7 as outputs
	DDRB |= (1 << PB4);				// PB4 as output for turn signal
	DDRB |= (1 << PB3);				// PB3 as output for steering beam
	DDRB |= (1 << PB2);				// PB3 as output for low beam
	DDRC &= ~(1 << PC4);			// PC4 as input for steering
	PORTC |= (1 << PC3);			// PC3 as input	for low beam
	PORTC |= (1 << PC2);			// PC2 as input for turn signal
	
	/* initially turn off led (active low) */
	PORTB |= (1 << PB4);
	
	timer_intr_init();				// init timer counter interrupts
	lcd_init();						// init LCD
	adc_init();						// init analog digital converter
	
	while (TRUE)
	{
		// wenn adc2 fertig auslesen -> setup steering, start adc1
		// wenn adc1 fertig auslesen -> setup velo, start adc2
		
		lcd_clear();
		
		if ((lcd_stat_reg & (1 << TURN_SIG))) {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("TS ON ");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("TS OFF");				// put value to LCD
		}
		
		if ((lcd_stat_reg & (1 << LOW_BEAM))) {
			lcd_set_cursor(0, 7);			// write to the fourth column of the first row
			lcd_text("LB ON ");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 7);			// write to the fourth column of the first row
			lcd_text("LB OFF");				// put value to LCD
		}
		
		if ((lcd_stat_reg & (1 << STEERING))) {
			lcd_set_cursor(0, 14);			// write to the fourth column of the first row
			lcd_text("T");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 14);			// write to the fourth column of the first row
			lcd_text("F");				// put value to LCD
		}
		
		AD_val = ADCH;
		int2ascii(AD_val, lcd_str);			// convert int to ascii representation
		lcd_set_cursor(1, 0);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		
		int2ascii(dim_val, lcd_str);			// convert int to ascii representation
		lcd_set_cursor(1, 4);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		
		_delay_ms(100);
	}
}

void timer_intr_init(void)
{
	/* Timer0 settings */
	TCCR0 = (1 << CS01);	// prescaler 8
	
	/* Timer1 settings */	
    /* Set the Timer/Counter1 to Fast PWM mode with ICR1 as top value (Mode 14) */
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);

    /* Set Compare Output Mode for channel B to non-inverting mode */
    TCCR1A |= (1 << COM1B1);
    TCCR1A &= ~(1 << COM1B0);
	
	/* Set the prescaler to 8 and start the timer */
	TCCR1B |= (1 << CS11);
	
    /* Set ICR1 for 1 kHz PWM frequency (16 bit) */
    ICR1 = 460;

    /* Set duty cycle for channel B (16 bit) (0 % duty = 0, 100 % = 460) */
    OCR1B = 460;
	
	/* Timer2 settings */
	/* enable fast PWM, with clear oc2 at compare match, prescaler 8 */
	TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21);		
	/* set duty cycle to 100 % since headlight has inverted logic (8 bit register) */
	OCR2 = 255;
	
	/* overflow interrupt */
	TIMSK |= (1 << TOIE1) | (1 << TOIE0);
	
	/* global interrupt enable */
	sei();	
}

void adc_init(void)
{
	ADMUX |= (1 << REFS0) | (1 << ADLAR) | (1 << MUX2);	// AVcc as reference, left adjust -> 8 bit result
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADFR);	// prescaler to 64, free running, enable adc, start conversion
	
	ADCSRA |= (1 << ADSC);	// start conversion
}

void turn_signal_processing(void) 
{
	static TurnSigState turn_sig_state = TS_IDLE;		// initial state is always idle
	static uint16_t ctr = 0, comf_ctr = 0, desc_ctr = 0;
	static uint8_t go_to_cont = FALSE;
	static uint8_t exit_cont = FALSE;
	
	/* turn signal state machine */
	switch (turn_sig_state) {
		case TS_IDLE:
			if (btn_stat_reg & (1 << TS_BTN)) {
				/* if button is pressed and valid, go to comfort mode */
				turn_sig_state = TS_COMF;
			
				/* initially turn on LED (active low) */
				PORTB &= ~(1 << PB4);
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
				
				/* only allow exit after loop (button condition is required so loop does not instantly start again) */
				if (exit_cont && !(btn_stat_reg & (1 << TS_BTN)))
				{
					ctrl_reg &= ~(1 << TURN_OFF_TS);	// unset turn off turn signal
								
					ctr = 0;
					turn_sig_state = TS_IDLE;
					exit_cont = FALSE;
					/* manually turn off led (active low) */
					PORTB |= (1 << PB4);
					lcd_stat_reg &= ~(1 << TURN_SIG);
				}
			}
			else {	
				ctr += 1;
			}
			
			/* check if either button is pressed again or steering is done*/
			if ((btn_stat_reg & (1 << TS_BTN)) || ctrl_reg & (1 << TURN_OFF_TS)) {
				exit_cont = TRUE;
			}
		
			break;
		default:
			/* if we somehow land here, reset turn signal state */
			turn_sig_state = TS_IDLE;
			break;
	}
}

void low_beam_processing(void)
{
	static LowBeamState low_beam_state = LB_IDLE;		// initial state is always idle
	static uint16_t ctr = 0;
	static uint8_t dim_cycl = 0;
	static uint8_t exit_cont = FALSE;
	
	/* turn signal state machine */
	switch (low_beam_state) {
		case LB_IDLE:
			if (btn_stat_reg & (1 << LB_BTN)) {
				/* if button is pressed and valid, start dimming */
				low_beam_state = LB_DIM;
			}
			break;
		case LB_DIM:
			if (dim_cycl < 10) {
				if (ctr > 100) {
					// increase oc1b
					dim_val -= 25;
					OCR2 = dim_val;
					OCR1B = dim_val;
					ctr = 0;
					dim_cycl += 1;
				}
				else {
					ctr += 1;
				}
			}
			else {
				low_beam_state = LB_ON;
				dim_val = 0;
				OCR2 = dim_val;
				OCR1B = dim_val;
				ctr = 0;
				dim_cycl = 0;
			}
			break;
		case LB_ON:
				lcd_stat_reg |= (1 << LOW_BEAM);
				
				/* only allow exit after loop (button condition is required so loop does not instantly start again) */
				if (exit_cont && !(btn_stat_reg & (1 << LB_BTN)))
				{							
					low_beam_state = LB_IDLE;
					exit_cont = FALSE;
					/* manually turn off led (active low) */
					OCR2 = 255;
					OCR1B = 460;
					dim_val = 0xFF;
					lcd_stat_reg &= ~(1 << LOW_BEAM);
				}
							
				/* check if either button is pressed again or steering is done*/
				if ((btn_stat_reg & (1 << LB_BTN))) {
					exit_cont = TRUE;
				}
			break;
		default:
			/* if we somehow land here, reset turn signal state */
			low_beam_state = LB_IDLE;
		break;
	}
}

void turn_signal_button_debounce(void)
{
	static ButtonState ts_btn_state = BTN_RELEASED;		// initial state is always button released
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
			break;
		default:
			/* if we somehow land here, reset button state */
			ts_btn_state = BTN_RELEASED;
			break;
	}
}

void low_beam_button_debounce(void)
{
	static ButtonState lb_btn_state = BTN_RELEASED;		// initial state is always button released
	static uint16_t ctr = 0;
	
	/* turn signal state machine */
	switch (lb_btn_state) {
		case BTN_RELEASED:
			if (!(PINC & (1 << PC3))) {
				/* if button is pressed, start debouncing */
				lb_btn_state = BTN_PRESSED_DB;
			}
			break;
		case BTN_PRESSED_DB:
			/* debouncing */
			if (ctr > DB_TIME_MS) {
				if (!(PINC & (1 << PC3))) {
					/* debounce successful -> set button state and go to pressed state*/
					btn_stat_reg |= (1 << LB_BTN);
					lb_btn_state = BTN_PRESSED;
				}
				else {
					/* debounce unsuccessful -> back to not pressed */
					lb_btn_state = BTN_RELEASED;
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
			if (PINC & (1 << PC3)) {
				/* if so, reset button state and go to released */
				lb_btn_state = BTN_RELEASED_DB;
			}
			break;
		case BTN_RELEASED_DB:
			/* button release debounce (prevents strange behavior)*/
			if (ctr > DB_TIME_MS) {
				if (PINC & (1 << PC3)) {
					/* debounce successful -> set button state and go to pressed state*/
					btn_stat_reg &= ~(1 << LB_BTN);
					lb_btn_state = BTN_RELEASED;
				}
				else {
					/* debounce unsuccessful -> back to pressed */
					lb_btn_state = BTN_PRESSED;
				}
				/* reset db counter */
				ctr = 0;
			}
			else {
				ctr += 1;
			}
			break;
		default:
			/* if we somehow land here, reset button state */
			lb_btn_state = BTN_RELEASED;
			break;
	}
}

void steering_processing(void)
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

ISR (TIMER0_OVF_vect)
{
	steering_processing();
}

ISR (TIMER1_OVF_vect)
{
	// TODO pwm 1 khz frequenz -> über icr1
	// OCR1B stellt nur duty cycle ein
	
	/* debounce routines */
	turn_signal_button_debounce();
	low_beam_button_debounce();
	
	/* ISR primarily used for processing */
	turn_signal_processing();
	low_beam_processing();
}

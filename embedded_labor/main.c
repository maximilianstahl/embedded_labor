/*
 * main.c
 *
 * Created: 06/06/2024 09:52:00
 * Author : mast274931
 */ 

/* F_CPU needs to be defined before delay is imported */
#define	F_CPU			3686400

#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRUE			0x01
#define FALSE			0x00

/* debounce time in milliseconds */
#define DB_TIME_MS		50	

/* defines for button status register */
#define TS_BTN			1
#define LB_BTN			2

/* defines for lcd status register */
#define TURN_SIG		1
#define LOW_BEAM		2
#define	CORNERING		3	
#define STEERING		4

/* defines for control register*/
#define STEER_DONE		1
#define TURN_OFF_TS		2
#define TS_ON			3
#define LOW_BEAM_ON		4		

/* defines for adc channels */
#define STEER_ADC_CH	4
#define VELO_ADC_CH		5

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

typedef enum {
	CL_IDLE = 1,	// idle state
	CL_DIM = 2,		// dimming mode
	CL_ON = 3		// on mode
} CorneringLightState;

/* macros to improve code readability */
#define IS_SET(reg, bit)		((reg) & (1 << (bit)))
#define SET_BIT(reg, bit)		((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)		((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)	((reg) ^= (1 << (bit)))

/* macros to set stepper motor direction */
#define SET_STEP_RIGHT()		(PORTC |= (1 << PC0))
#define SET_STEP_LEFT()			(PORTC &= ~(1 << PC0))

volatile uint8_t btn_stat_reg = 0x00;	// register for button statuses
volatile uint8_t lcd_stat_reg = 0x00;	// register to tell LCD what to display
volatile uint8_t ctrl_reg = 0x00;		// register for general control exchange

void timer_intr_init(void);
void adc_init(void);

void turn_signal_button_debounce(void);
void low_beam_button_debounce(void);

void turn_signal_processing(void);
void low_beam_processing(void);
void cornering_light_processing(void);
void stepper_motor_processing(void);

void steering_processing(uint8_t *steering_val);
void velocity_processing(uint8_t *velo_val);

void read_adcs(uint8_t *steering_val, uint8_t *velo_val);

// TODO: there is a better way
volatile uint8_t velo_value = 0;
volatile uint8_t steer_value = 0;

int main(void)
{
	uint8_t steering_val = 0, velo_val = 0;		// define vars for steering and velocity adc values
	char lcd_str[17];

	DDRD |= 0xFC;					// set bits 2...7 as outputs
	DDRB |= (1 << PB5);				// PB5 as output for stepper motor clock
	DDRB |= (1 << PB4);				// PB4 as output for turn signal
	DDRB |= (1 << PB3);				// PB3 as output for steering beam
	DDRB |= (1 << PB2);				// PB3 as output for low beam
	DDRC &= ~(1 << PC5);			// PC5 as input for velocity
	DDRC &= ~(1 << PC4);			// PC4 as input for steering
	PORTC |= (1 << PC3);			// PC3 as input	pull up for low beam
	PORTC |= (1 << PC2);			// PC2 as input pull up for turn signal
	DDRC |= (1 << PC0);				// PC0 as output for stepper motor cw/ccw
	
	PORTB |= (1 << PB4);			// initially turn off led (active low)
	PORTB &= ~(1 << PB5);
	
	timer_intr_init();				// init timer counter interrupts
	lcd_init();						// init LCD
	adc_init();						// init analog digital converters
	
	while (TRUE)
	{
		/* read adc values and process these values */
		read_adcs(&steering_val, &velo_val);
		steering_processing(&steering_val);
		velocity_processing(&velo_val);
		
		/* clear LCD before writing to it */
		lcd_clear();
		
		if ((lcd_stat_reg & (1 << TURN_SIG))) {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("TS T");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_text("TS F");				// put value to LCD
		}
		
		if ((lcd_stat_reg & (1 << LOW_BEAM))) {
			lcd_set_cursor(0, 5);			// write to the fourth column of the first row
			lcd_text("LB T");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 5);			// write to the fourth column of the first row
			lcd_text("LB F");				// put value to LCD
		}
		
		if ((lcd_stat_reg & (1 << CORNERING))) {
			lcd_set_cursor(0, 10);			// write to the fourth column of the first row
			lcd_text("CL T");				// put value to LCD
		}
		else {
			lcd_set_cursor(0, 10);			// write to the fourth column of the first row
			lcd_text("CL F");				// put value to LCD
		}
		
		int2ascii(steering_val, lcd_str);			// convert int to ascii representation
		lcd_set_cursor(1, 0);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		
		int2ascii(velo_val, lcd_str);			// convert int to ascii representation
		lcd_set_cursor(1, 4);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		
		_delay_ms(100);
	}
}

void timer_intr_init(void)
{
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
	TIMSK |= (1 << TOIE1);// | (1 << TOIE0);
	
	/* global interrupt enable */
	sei();	
}

void adc_init(void)
{
	ADMUX |= (1 << REFS0) | (1 << ADLAR);	// AVcc as reference, left adjust -> 8 bit result
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);	// prescaler to 64, enable adc
}

void read_adcs(uint8_t *steering_val, uint8_t *velo_val)
{
	static uint8_t curr_adc_ch = STEER_ADC_CH;
	
	/* read value and change channel if conversion is finished */
	if (!(ADCSRA & (1<<ADSC))) {
		if (curr_adc_ch == STEER_ADC_CH) {
			*steering_val = ADCH;
			curr_adc_ch = VELO_ADC_CH;
		}
		else if (curr_adc_ch == VELO_ADC_CH) {
			*velo_val = ADCH;
			curr_adc_ch = STEER_ADC_CH;
		}
		else {
			/* we should not land here, if we do -> reset to initial value */
			curr_adc_ch = STEER_ADC_CH;
		}

		ADMUX = (ADMUX & 0xF8) | curr_adc_ch;
		ADCSRA |= (1<<ADSC);	// Start single conversion
	}
}

void turn_signal_processing(void) 
{
	static TurnSigState turn_sig_state = TS_IDLE;		// initial state is always idle
	static uint16_t ctr = 0, comf_ctr = 0, desc_ctr = 0;
	static uint8_t go_to_cont = FALSE, exit_cont = FALSE;
	
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
					/* set control reg to let steering light know */
					ctrl_reg |= (1 << TS_ON);
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
					ctrl_reg &= ~(1 << TS_ON);		/* unset control reg to let steering light know */
					
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
	static uint16_t ctr = 0, dim_val = 460;
	static uint8_t dim_cycl = 0;

	void check_turn_off()
	{
		/* function to prevent duplicated code in low beam state machine */
		static uint8_t exit_cont = FALSE;
		
		/* only allow exit after loop (button condition is required so loop does not instantly start again) */
		if (exit_cont && !(btn_stat_reg & (1 << LB_BTN))) {
			low_beam_state  = LB_IDLE;
			exit_cont = FALSE;
			dim_cycl = 0;
			/* manually turn off led (active low) */
			dim_val = 460;
			OCR1B = dim_val;
			lcd_stat_reg &= ~(1 << LOW_BEAM);
			/* unset control reg to let steering light know */
			ctrl_reg &= ~(1 << LOW_BEAM_ON);
		}
		
		/* check if button is pressed again*/
		if (btn_stat_reg & (1 << LB_BTN)) {
			exit_cont = TRUE;
		}
	}
	
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
					/* decrease OCR1B every 100 ms */ // TODO supposed to take 5s not 1s
					dim_val -= 46;
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
				OCR1B = dim_val;
				ctr = 0;
				dim_cycl = 0;
			}
			// check_turn_off();	// TODO does not work ???
			break;
		case LB_ON:
			/* set LCD reg to let display know */
			lcd_stat_reg |= (1 << LOW_BEAM);
			/* set control reg to let steering light know */
			ctrl_reg |= (1 << LOW_BEAM_ON);
			check_turn_off();
			break;
		default:
			/* if we somehow land here, reset turn signal state */
			low_beam_state = LB_IDLE;
		break;
	}
}

void cornering_light_processing(void)
{
	static CorneringLightState corn_light_state = CL_IDLE;		// initial state is always idle
	static uint16_t ctr = 0;
	static uint8_t dim_cycl = 0, dim_val = 255;

	/* turn signal state machine */
	switch (corn_light_state) {
		case CL_IDLE:
			/* if either ts or steering */
			if ((ctrl_reg & (1 << LOW_BEAM_ON)) && (velo_value < 20) && (ctrl_reg & (1 << TS_ON))) {
				corn_light_state = CL_DIM;
			}
			break;
		case CL_DIM:
			if (dim_cycl < 10) {
				if (ctr > 100) {
					/* decrease OCR2 every 100 ms */	// TODO supposed to take 5s not 1s
					dim_val -= 25;
					OCR2 = dim_val;
					ctr = 0;
					dim_cycl += 1;
				}
				else {
					ctr += 1;
				}
			}
			else {
				corn_light_state = CL_ON;
				dim_val = 0;
				OCR2 = dim_val;
				ctr = 0;
				dim_cycl = 0;
			}
			break;
		case CL_ON:
			lcd_stat_reg |= (1 << CORNERING);
		
			/* only allow exit after loop (button condition is required so loop does not instantly start again) */
			if (!(ctrl_reg & (1 << LOW_BEAM_ON)) || !(velo_value < 20) || !(ctrl_reg & (1 << TS_ON)))
			{
				corn_light_state = CL_IDLE;
				/* manually turn off led (active low) */
				dim_val = 255;
				OCR2 = dim_val;
				lcd_stat_reg &= ~(1 << CORNERING);
			}
			break;
		default:
			/* if we somehow land here, reset turn signal state */
			corn_light_state = CL_IDLE;
			break;
	}
}

void stepper_motor_processing(void)
{
	static uint16_t steps = 0;
	static uint8_t ctr = 0;
	
	if (steps < 400) {
		if (ctr > 5) {
			PORTB ^= (1 << PB5);
			ctr = 0;
			steps += 1;
		}
		else {
			ctr += 1;
		}
	}
	else {
		PORTC ^= (1 << PC0);
		steps = 0;
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

void steering_processing(uint8_t *steering_val)
{
	// TODO rework, quite ugly (?)
	// TODO add greater than 10 deg detection for cornering light
	static uint8_t steering_initiated = FALSE;	// var to tell if steering process is initiated

	/* steering processing */
	if (!steering_initiated) {
		if (!(((127 - 42) < *steering_val) && (*steering_val < (127 + 42)))) {
			/* steering process is initiated -> not -10� < lw < 10� */
			steering_initiated = TRUE;
			lcd_stat_reg &= ~(1 << STEERING);	// unset steering done
				
		}
		else {
			ctrl_reg &= ~(1 << TURN_OFF_TS);	// unset turn off turn signal
		}
	}
	else {
		if (((127 - 42) < *steering_val) && (*steering_val < (127 + 42))) {
			/* steering done -> -10� < lw < 10� */
			steering_initiated = FALSE;
			lcd_stat_reg |= (1 << STEERING);	// set steering done
			ctrl_reg |= (1 << TURN_OFF_TS);		// set turn off turn signal
		}
	}
}

void velocity_processing(uint8_t *velo_val)
{
	// TODO rework, quite ugly (?)
	velo_value = *velo_val;
}

ISR (TIMER1_OVF_vect)
{
	/* debounce routines */
	turn_signal_button_debounce();
	low_beam_button_debounce();
	
	/* ISR primarily used for processing */
	turn_signal_processing();
	low_beam_processing();
	cornering_light_processing();
	
	/* stepper test code */
	if (FALSE) {
		stepper_motor_processing();
	}
}



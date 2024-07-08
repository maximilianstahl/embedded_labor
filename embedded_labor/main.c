/*
 * main.c
 *
 * Created: 06/06/2024 09:52:00
 * Author : mast274931
 */ 

#define	F_CPU			3686400

#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* debounce time in milliseconds */
#define DB_TIME_MS				50	

// TODO evaluate speed
/* stepper motor edge duration */
#define EDGE_DUR_MS				10

/* defines for button status register */
#define TS_BTN					1
#define LB_BTN					2

/* defines for lcd status register */
#define TURN_SIG				1
#define LOW_BEAM				2
#define	CORNERING				3	

/* defines for control register*/
#define STEER_DONE				1
#define STEER_TS_OFF			2
#define TS_ON					3
#define LOW_BEAM_ON				4		

/* defines for adc channels */
#define STEER_ADC_CH			4
#define VELO_ADC_CH				5

/* true and false to improve code readability */
#define TRUE					0x01
#define FALSE					0x00

/* macros to improve code readability */
#define BIT_IS_SET(reg, bit)	((reg) & (1 << (bit)))
#define SET_BIT(reg, bit)		((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)		((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)	((reg) ^= (1 << (bit)))

/* macros to set stepper motor direction */
#define SET_STEP_DIR_R()		SET_BIT(PORTC, PC0)
#define SET_STEP_DIR_L()		CLEAR_BIT(PORTC, PC0)

typedef enum {
	BTN_RELEASED = 1,	// released state
	BTN_PRESSED = 2,	// pressed state
	BTN_PRESSED_DB = 3,	// pressed debounce state
	BTN_RELEASED_DB = 4	// released debounce state
} ButtonState;

typedef enum {
	TS_IDLE = 1,		// idle state -> waiting for button signal
	TS_COMF = 2,		// comfort mode
	TS_CONT = 3,		// continuous mode
	TS_TURN_OFF = 4		// turn off state
} TurnSignalState;

typedef enum {
	LB_IDLE = 1,		// idle state -> waiting for button signal
	LB_DIM = 2,			// dimming mod
	LB_ON = 3,			// on mode
	LB_TURN_OFF = 4		// turn off state
} LowBeamState;

typedef enum {
	CL_IDLE = 1,		// idle state
	CL_DIM = 2,			// dimming mode
	CL_ON = 3,			// on mode
	CL_TURN_OFF = 4		// turn off state
} CorneringLightState;

volatile uint8_t btn_stat_reg = 0x00;	// register for button statuses
volatile uint8_t lcd_stat_reg = 0x00;	// register to tell LCD what to display
volatile uint8_t ctrl_reg = 0x00;		// register for general control exchange

/* init functions */
void timer_intr_init(void);
void adc_init(void);
uint8_t stepper_motor_calib(void);

/* calculation and conversion functions */
void read_adcs(uint8_t *steering_ang, uint8_t *velo_val);
void steering_conversion(uint8_t *steering_ang);
void velocity_conversion(uint8_t *velo_val);
int16_t swivel_calc(void);
uint8_t step(void);

/* debounce functions */
void turn_signal_button_debounce(void);
void low_beam_button_debounce(void);

/* processing functions */
void turn_signal_processing(void);
void low_beam_processing(void);
void cornering_light_processing(void);
void stepper_motor_processing(int16_t swivel_angle);
void steering_processing(void);

// TODO: there is a better way + think about variable renaming, quite some steering steer ang angle
/* global variables to exchange data between main and ISR */
volatile uint8_t velo_value = 0;
volatile int8_t steer_angle = 0;
volatile uint8_t old_edge;

volatile int16_t curr_pos = 0;
volatile int16_t swivel = 0;

int main(void)
{
	uint8_t steer_ang = 0, velo_val = 0;		// define vars for steering and velocity adc values
	char lcd_str[17];

	DDRD |= 0xFC;					// set bits 2...7 as outputs
	
	SET_BIT(DDRB, PB5);				// PB5 as output for stepper motor clock
	SET_BIT(DDRB, PB4);				// PB4 as output for turn signal
	SET_BIT(DDRB, PB3);				// PB3 as output for steering beam
	SET_BIT(DDRB, PB2);				// PB3 as output for low beam
	
	CLEAR_BIT(DDRC, PC5);			// PC5 as input for velocity
	CLEAR_BIT(DDRC, PC4);			// PC4 as input for steering
	SET_BIT(PORTC, PC3);			// PC3 as input	pull up for low beam
	SET_BIT(PORTC, PC2);			// PC2 as input pull up for turn signal
	CLEAR_BIT(DDRC, PC1);			// PC1 as input for stepper motor hall sensor out
	SET_BIT(DDRC, PC0);				// PC0 as output for stepper motor cw/ccw
	
	SET_BIT(PORTB, PB4);				// initially turn off turn signal (active low)
	old_edge = BIT_IS_SET(PINC, PC1);	// get old edge for calibration comparison
	
	/* set stepper direction to opposite of current position (so it always moves back to middle) */ 
	PORTC = BIT_IS_SET(PINC, PC1) ? SET_STEP_DIR_L() : SET_STEP_DIR_R();
	
	timer_intr_init();				// init timer counter interrupts
	lcd_init();						// init LCD
	adc_init();						// init analog digital converters 

	while (TRUE)
	{
		/* read adc values and process these values */
		read_adcs(&steer_ang, &velo_val);
		
		/* convert adc values into corresponding steering angle and velocity value */
		steering_conversion(&steer_ang);
		velocity_conversion(&velo_val);
		
		/* process steering */
		steering_processing();
		
		/* clear LCD before writing to it */
		lcd_clear();
		
		/* turn signal symbol print */
		if (BIT_IS_SET(lcd_stat_reg, TURN_SIG)) {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_write(TS_ON_L_SYM);			// display turn signal left on symbol
		}
		else {
			lcd_set_cursor(0, 0);			// write to the first column of the first row
			lcd_write(TS_OFF_L_SYM);		// display turn signal left on symbol
		}

		/* turn signal right is always off as we only have the left headlight ;-) */
		lcd_set_cursor(0, 2);				// write to the second column of the first row
		lcd_write(TS_OFF_R_SYM);			// display turn signal right off symbol

		/* steering angle print */
		int8t2ascii(steer_angle, lcd_str);	// convert int to ascii representation
		lcd_set_cursor(0, 5);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		lcd_set_cursor(0, 9);				// write to the first column of the second row
		lcd_write(DEG_SYM);					// display degree symbol
		
		/* debug print for current headlight position */
		//lcd_set_cursor(0, 5);				// write to the first column of the second row
		//lcd_text("CP: ");					// put value to LCD
		//int16t2ascii(curr_pos, lcd_str);	// convert int to ascii representation
		//lcd_set_cursor(0, 9);				// write to the first column of the second row
		//lcd_text(lcd_str);					// put value to LCD

		/* low beam symbol print */
		if (BIT_IS_SET(lcd_stat_reg, LOW_BEAM)) {
			lcd_set_cursor(1, 0);			// write to the second column of the first row
			lcd_write(LB_ON_SYM);			// display low beam on symbol
		}
		else {
			lcd_set_cursor(1, 0);			// write to the second column of the first row
			lcd_write(LB_OFF_SYM);			// display low beam off symbol
		}
		
		/* cornering light symbol print */
		if (BIT_IS_SET(lcd_stat_reg, CORNERING)) {
			lcd_set_cursor(1, 2);			// write to the second column of the first row
			lcd_write(CL_ON_SYM);			// display cornering light on symbol
		}
		else {
			lcd_set_cursor(1, 2);			// write to the second column of the first row
			lcd_write(CL_OFF_SYM);			// display cornering light off symbol
		}

		/* velocity value print */
		uint8t2ascii(velo_value, lcd_str);	// convert unsigned int to ascii representation
		lcd_set_cursor(1, 6);				// write to the first column of the second row
		lcd_text(lcd_str);					// put value to LCD
		lcd_set_cursor(1, 10);				// write to the first column of the second row
		lcd_text("km/h");					// put value to LCD
		
		/* debug print for current swivel */
		//lcd_set_cursor(1, 5);				// write to the first column of the second row
		//lcd_text("SW: ");					// put value to LCD
		//int16t2ascii(swivel, lcd_str);		// convert int to ascii representation
		//lcd_set_cursor(1, 9);				// write to the first column of the second row
		//lcd_text(lcd_str);
				
		_delay_ms(100);
	}
}

void timer_intr_init(void)
{
	/* Timer1 settings */	
    /* Set the Timer/Counter1 to Fast PWM mode with ICR1 as top value (Mode 14) */
    SET_BIT(TCCR1A, WGM11);
    SET_BIT(TCCR1B, WGM12);
	SET_BIT(TCCR1B, WGM13);

    /* Set Compare Output Mode for channel B to non-inverting mode */
    SET_BIT(TCCR1A, COM1B1);
    CLEAR_BIT(TCCR1A, COM1B0);
	
	/* Set the prescaler to 8 and start the timer */
	SET_BIT(TCCR1B, CS11);
	
    /* Set ICR1 for 1 kHz PWM frequency (16 bit) */
    ICR1 = 460;

    /* Set duty cycle for channel B (16 bit) (0 % duty = 0, 100 % = 460) */
    OCR1B = 460;
	
	/* Timer2 settings */
	/* enable fast PWM, with clear oc2 at compare match, prescaler 8 */
	SET_BIT(TCCR2, WGM21);
	SET_BIT(TCCR2, WGM20);
	SET_BIT(TCCR2, COM21);
	SET_BIT(TCCR2, CS21);

	/* set duty cycle to 100 % since headlight has inverted logic (8 bit register) */
	OCR2 = 255;
	
	/* overflow interrupt */
	SET_BIT(TIMSK, TOIE1);
	
	/* global interrupt enable */
	sei();	
}

void adc_init(void)
{
	/* AVcc as reference, left adjust -> 8 bit result */
	SET_BIT(ADMUX, REFS0);
	SET_BIT(ADMUX, ADLAR);

	/* enable adc, prescaler to 64 */
	SET_BIT(ADCSRA, ADEN);
	SET_BIT(ADCSRA, ADPS2);
	SET_BIT(ADCSRA, ADPS1);
}

uint8_t step(void)
{
	/* return TRUE: step done, return FALSE: step in process */
	static uint16_t ctr = 0;
	
	if (ctr == (EDGE_DUR_MS / 2)) {
		/* set edge high after half duration */
		SET_BIT(PORTB, PB5);
		ctr += 1;
	}
	else if (ctr > EDGE_DUR_MS) {
		/* set edge low after full duration */
		CLEAR_BIT(PORTB, PB5);
		ctr = 0;
		return TRUE;
	}
	else {
		ctr += 1;
	}
	
	return FALSE;
}

uint8_t stepper_motor_calib(void)
{
	static uint8_t calib_done = FALSE;

	if (!calib_done) {
		/* calib only does stepping, direction is already set while init */
		step();
	
		/* check if middle position is reached -> edge detected */
		if (old_edge != BIT_IS_SET(PINC, PC1)) {
			calib_done = TRUE;
		}
	}

	return calib_done;
}

void read_adcs(uint8_t *steering_ang, uint8_t *velo_val)
{
	static uint8_t curr_adc_ch = STEER_ADC_CH;
	
	/* read value and change channel if conversion is finished */
	if (!BIT_IS_SET(ADCSRA, ADSC)) {
		if (curr_adc_ch == STEER_ADC_CH) {
			*steering_ang = ADCH;
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
		
		/* switch channel and start single conversion again */
		ADMUX = (ADMUX & 0xF8) | curr_adc_ch;
		SET_BIT(ADCSRA, ADSC);
	}
}

void turn_signal_processing(void) 
{
	static TurnSignalState turn_sig_state = TS_IDLE;		// initial state is always idle
	static uint16_t ctr = 0, comf_ctr = 0, desc_ctr = 0;
	static uint8_t go_to_cont = FALSE, exit_cont = FALSE, allow_turn_off = FALSE;
	
	/* turn signal state machine */
	switch (turn_sig_state) {
		case TS_IDLE:
			if (BIT_IS_SET(btn_stat_reg, TS_BTN)) {
				/* if button is pressed and valid, go to comfort mode */
				turn_sig_state = TS_COMF;
				
				/* initially turn on LED (active low) */
				CLEAR_BIT(PORTB, PB4);
				SET_BIT(lcd_stat_reg, TURN_SIG);
			}
			break;
		case TS_COMF:
			/* only allow check during the first second */
			if ((comf_ctr < (1000 - DB_TIME_MS)) && (BIT_IS_SET(btn_stat_reg, TS_BTN))) {
				desc_ctr += 1;
			
				/* check if button is held for 1 second minus debounce time */
				if (desc_ctr >= (1000 - DB_TIME_MS)) {
					go_to_cont = TRUE;
					/* set control reg to let steering light know */
					SET_BIT(ctrl_reg, TS_ON);
				}
			}
		
			/* 5 because initial state is on, so we just need 5 instead of 6 led toggles */
			if (comf_ctr < 5) {
				if (ctr > 500) {
					comf_ctr += 1;
					ctr = 0;
					
					/* toggle turn signal and status register */
					TOGGLE_BIT(PORTB, PB4);
					TOGGLE_BIT(lcd_stat_reg, TURN_SIG);
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
				/* toggle turn signal and status register */
				TOGGLE_BIT(PORTB, PB4);
				TOGGLE_BIT(lcd_stat_reg, TURN_SIG);
				ctr = 0;
				
				/* only allow exit after loop (button condition is required so loop does not instantly start again) */
				if (exit_cont && !(BIT_IS_SET(btn_stat_reg, TS_BTN))) {
					turn_sig_state = TS_TURN_OFF;
					exit_cont = FALSE;
				}
			}
			else {	
				ctr += 1;
			}
			
			/* check if either button is pressed again or steering is done */
			if (BIT_IS_SET(btn_stat_reg, TS_BTN) || BIT_IS_SET(ctrl_reg, STEER_TS_OFF)) {
				/* clear allow turn off and exit continues after next execution */
				exit_cont = TRUE;
			}
			
			// TODO why does this not work?
			///* button has to be released in order to turn continuous mode off (so holding the button for longer than 3 s does not turn the turn signal off after releasing)*/
			//if (!BIT_IS_SET(btn_stat_reg, TS_BTN)) {
				//allow_turn_off = TRUE;
			//}
			//
			///* check if either button is pressed again or steering is done */
			//if (allow_turn_off && (BIT_IS_SET(btn_stat_reg, TS_BTN) || BIT_IS_SET(ctrl_reg, STEER_TS_OFF))) {
				///* clear allow turn off and exit continues after next execution */
				//exit_cont = TRUE;
				//allow_turn_off = FALSE;
			//}
			break;
		case TS_TURN_OFF:
			CLEAR_BIT(ctrl_reg, STEER_TS_OFF);	// unset turn off turn signal
			/* unset control reg to let steering light know */
			CLEAR_BIT(ctrl_reg, TS_ON);		
			/* manually turn off turn signal (active low) and clear lcd bit */
			SET_BIT(PORTB, PB4);
			CLEAR_BIT(lcd_stat_reg, TURN_SIG);

			if (!BIT_IS_SET(btn_stat_reg, TS_BTN)) {	
				turn_sig_state = TS_IDLE;
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
	static uint8_t dim_cycl = 0, allow_turn_off = FALSE;
	
	/* turn signal state machine */
	switch (low_beam_state) {
		case LB_IDLE:
			if (BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				/* if button is pressed and valid, start dimming */
				low_beam_state = LB_DIM;
			}
			break;
		case LB_DIM:
			/* set LCD reg to let display know */
			SET_BIT(lcd_stat_reg, LOW_BEAM);
			/* set control reg to let steering light know */
			SET_BIT(ctrl_reg, LOW_BEAM_ON);

			/* decrease OCR1B every 100 ms -> (50 - 1) because final step is done in else condition to have consistant dimming and avoid floating point operations */
			if (dim_cycl < (50 - 1)) {
				if (ctr > 100) {
					dim_val -= 9;
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
			
			/* once button is released, allow turn off state -> needed to prevent instant exit */
			if (!BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				allow_turn_off = TRUE;
			}

			/* check if button is pressed again, if so turn lb off */
			if (allow_turn_off && BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				low_beam_state = LB_TURN_OFF;
				allow_turn_off = FALSE;
			}
			break;
		case LB_ON:
			/* once button is released, allow turn off state -> needed to prevent instant exit */
			if (!BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				allow_turn_off = TRUE;
			}

			/* check if button is pressed again, if so turn lb off */
			if (allow_turn_off && BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				low_beam_state = LB_TURN_OFF;
				allow_turn_off = FALSE;
			}
			break;
		case LB_TURN_OFF:
			/* manually turn off led (active low) */
			OCR1B = dim_val;
			dim_val = 460;
			dim_cycl = 0;

			CLEAR_BIT(lcd_stat_reg, LOW_BEAM);
			/* unset control reg to let steering light know */
			CLEAR_BIT(ctrl_reg, LOW_BEAM_ON);

			/* button condition is required so loop does not instantly start again) */
			if (!BIT_IS_SET(btn_stat_reg, LB_BTN)) {
				low_beam_state = LB_IDLE;
			}
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
			if (BIT_IS_SET(ctrl_reg, LOW_BEAM_ON) && (velo_value <= 20)) {
				if ((steer_angle < -10) || BIT_IS_SET(ctrl_reg, TS_ON)) {
					corn_light_state = CL_DIM;
				}
			}
			break;
		case CL_DIM:
			SET_BIT(lcd_stat_reg, CORNERING);
			
			/* decrease OCR2 every 100 ms -> (50 - 1) because final step is done in else condition to have consistant dimming and avoid floating point operations */
			if (dim_cycl < (50 - 1)) {
				if (ctr > 100) {
					dim_val -= 5;
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

			/* check if cornering light should be turned off again */
			if (!BIT_IS_SET(ctrl_reg, LOW_BEAM_ON) || (velo_value > 20) || ((steer_angle > -5) && !BIT_IS_SET(ctrl_reg, TS_ON))) {
				corn_light_state = CL_TURN_OFF;
			}
			break;
		case CL_ON:
			/* check if cornering light should be turned off again */
			if (!BIT_IS_SET(ctrl_reg, LOW_BEAM_ON) || (velo_value > 20) || ((steer_angle > -5) && !BIT_IS_SET(ctrl_reg, TS_ON))) {
				corn_light_state = CL_TURN_OFF;
			}
			break;
		case CL_TURN_OFF:
			/* manually turn off led (active low) */
			dim_val = 255;
			OCR2 = dim_val;
			dim_cycl = 0;

			CLEAR_BIT(lcd_stat_reg, CORNERING);

			corn_light_state = CL_IDLE;
			break;
		default:
			/* if we somehow land here, reset turn signal state */
			corn_light_state = CL_IDLE;
			break;
	}
}

void stepper_motor_processing(int16_t swivel_angle)
{	
	/* input is swivel angle with comma shifted, so 11.5 is 115 (to calc steps more precise) */
	static int16_t current_pos = 0;
	
	if (current_pos != swivel_angle) {
		if (current_pos < swivel_angle) {
			SET_STEP_DIR_L();
			current_pos += step();
		}
		else if (current_pos > swivel_angle) {
			SET_STEP_DIR_R();
			current_pos -= step();
		}
	}
	
	// TODO remove debug prints after stepper works
	curr_pos = current_pos;
	swivel = swivel_angle;
}

void turn_signal_button_debounce(void)
{
	static ButtonState ts_btn_state = BTN_RELEASED;		// initial state is always button released
	static uint16_t ctr = 0;
	
	/* turn signal state machine */
	switch (ts_btn_state) {
		case BTN_RELEASED:
			if (!BIT_IS_SET(PINC, PC2)) {
				/* if button is pressed, start debouncing */
				ts_btn_state = BTN_PRESSED_DB;
			}
			break;
		case BTN_PRESSED_DB:
			/* debouncing */
			if (ctr > DB_TIME_MS) {
				if (!BIT_IS_SET(PINC, PC2)) {
					/* debounce successful -> set button state and go to pressed state */
					SET_BIT(btn_stat_reg, TS_BTN);
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
			if (BIT_IS_SET(PINC, PC2)) {
				/* if so, reset button state and go to released */
				ts_btn_state = BTN_RELEASED_DB;
			}
			break;
		case BTN_RELEASED_DB:
			/* button release debounce (prevents strange behavior) */
			if (ctr > DB_TIME_MS) {
				if (BIT_IS_SET(PINC, PC2)) {
					/* debounce successful -> set button state and go to released state */
					CLEAR_BIT(btn_stat_reg, TS_BTN);
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
			if (!BIT_IS_SET(PINC, PC3)) {
				/* if button is pressed, start debouncing */
				lb_btn_state = BTN_PRESSED_DB;
			}
			break;
		case BTN_PRESSED_DB:
			/* debouncing */
			if (ctr > DB_TIME_MS) {
				if (!BIT_IS_SET(PINC, PC3)) {
					/* debounce successful -> set button state and go to pressed state */
					SET_BIT(btn_stat_reg, LB_BTN);
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
			if (BIT_IS_SET(PINC, PC3)) {
				/* if so, reset button state and go to released */
				lb_btn_state = BTN_RELEASED_DB;
			}
			break;
		case BTN_RELEASED_DB:
			/* button release debounce (prevents strange behavior) */
			if (ctr > DB_TIME_MS) {
				if (BIT_IS_SET(PINC, PC3)) {
					/* debounce successful -> set button state and go to released state */
					CLEAR_BIT(btn_stat_reg, LB_BTN);
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

void steering_conversion(uint8_t *steer_ang)
{
	/* transform potentiometer value into linear steering characteristics */
	if (*steer_ang < 255) {
		steer_angle = (*steer_ang * 60) / 255 - 30;
	}
	else {
		steer_angle = 30;
	}
}

void velocity_conversion(uint8_t *velo_val)
{
	/* transform potentiometer value into linear velocity characteristics */
	if (*velo_val < 255) {
		velo_value = (*velo_val * 130) / 255  ;
	}
	else {
		velo_value = 130;
	}
}

void steering_processing(void)
{
	/* function to tell whether steering process to the left is initiated, done or non existing (only process left as we only control the left headlight) */
	static uint8_t steering_initiated = FALSE;

	/* steering processing */
	if (!steering_initiated) {
		if (!(-10 < steer_angle)) {
			/* steering process is initiated -> not -10° < lw */
			steering_initiated = TRUE;
		}
		else {
			/* clear turn off turn signal bit */
			CLEAR_BIT(ctrl_reg, STEER_TS_OFF);
		}
	}
	else {
		if (-5 < steer_angle) {
			/* steering done -> -5° < lw (hysterical behavior) */
			steering_initiated = FALSE;
			/* set turn off turn signal bit */
			SET_BIT(ctrl_reg, STEER_TS_OFF);
		}
	}
}

int16_t swivel_calc(void)
{
	/* returns swivel value with comma shifted to the left, so 7.5 is 75 (to be more precise with step calc) */
	int16_t swivel = 0;
	
	/* swivel calc only between 30 and 120 kmh, else stays */
	if ((velo_value >= 30) && (velo_value <= 120)) {
		swivel = (steer_angle * 10 * (90 - (velo_value - 30))) / 90;

		/* check if swivel angle is greater than +-15 (= max swivel)*/
		if (swivel > 150) {
			swivel = 150;
		}
		else if (swivel < -150) {
			swivel = -150;
		}

		/* if swivel angle is positive, we turn right, therefore divide angle by 2 */
		if (swivel >= 0) {
			swivel /= 2;
		}
	}
	
	return swivel;
}

ISR (TIMER1_OVF_vect)
{
	/* ISR for time critical operations */
	static uint8_t calib_done = FALSE;
	
	/* debounce routines */
	turn_signal_button_debounce();
	low_beam_button_debounce();
	
	/* processing routines */
	turn_signal_processing();
	low_beam_processing();
	cornering_light_processing();

	// if cond for developing
	if (FALSE) {
		if (calib_done) {
			stepper_motor_processing(swivel_calc());
		}
		else {
			calib_done = stepper_motor_calib();
		}
	} 
}


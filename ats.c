/*
 * ATS W2R.c
 *
 * Created: 12.04.2022 15:00:00
 * v. 1.0
 *  Author: Vadim Kulakov, vad7@yahoo.com
 * 
 * ATTiny13A
 *
 */ 
#define F_CPU 4700000UL
// Fuses(0=active): BODLEVEL = 2.7V (BODLEVEL[1:0] = 01), RSTDISBL=0, CKDIV8=0, CKSEL[1:0]=01 (4.8MHz), SUT[1:0]=01
// MK power drawing 2.8 mA

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define RELAY_LINE_PIN	(1<<PORTB4)
#define RELAY_LINE_ON	PORTB |= RELAY_LINE_PIN; 
#define RELAY_LINE_OFF	PORTB &= ~RELAY_LINE_PIN
#define RELAY_LINE_SW	PORTB ^= RELAY_LINE_PIN
#define RELAY_LINE_ST	(PORTB & RELAY_LINE_PIN)

#define RELAY_GEN_PIN	(1<<PORTB3)
#define RELAY_GEN_ON	PORTB |= RELAY_GEN_PIN
#define RELAY_GEN_OFF	PORTB &= ~RELAY_GEN_PIN
#define RELAY_GEN_SW	PORTB ^= RELAY_GEN_PIN
#define RELAY_GEN_ST	(PORTB & RELAY_GEN_PIN)

#define LED_LINE_PIN	(1<<PORTB0)		// RED
#define LED_LINE_ON		PORTB |= LED_LINE_PIN
#define LED_LINE_OFF	PORTB &= ~LED_LINE_PIN
#define LED_LINE_SW		PORTB ^= LED_LINE_PIN

#define LED_GEN_PIN		(1<<PORTB2)
#define LED_GEN_ON		PORTB |= LED_GEN_PIN
#define LED_GEN_OFF		PORTB &= ~LED_GEN_PIN
#define LED_GEN_SW		PORTB ^= LED_GEN_PIN

#define ON_LINE_PIN		(1<<PORTB5)
#define ON_LINE_PINT	(1<<PCINT5)
#define ON_LINE_PORT	PINB

#define ON_GEN_PIN		(1<<PORTB1)
#define ON_GEN_PINT	// INT0
#define ON_GEN_PORT		PINB

uint8_t timer_10ms = 0;
uint8_t timer_100ms = 0;
uint8_t timer = 0;
uint8_t line_pin = 0;
uint8_t line_active = 0;
uint8_t gen_active = 0;
uint8_t repeat_line_cnt = 0;

typedef enum {
	WP_UNKNOWN = 0,
	WP_LINE,
	WP_GEN
} What_pos;
What_pos pos = WP_UNKNOWN;

typedef enum {
	WD_NONE = 0,
	WD_RELAY_LINE,
	WD_RELAY_GEN,
	WD_DELAY_SWITCH_TO_LINE
} What_doing;
What_doing doing = WD_NONE;

struct _EEPROM {
	uint8_t timer_value;			// =Fclk/prescaller/Freq - 1
	uint8_t relay_on_time;			// *0.1 sec
	uint8_t relay_gen_always_on;	// dont turn off relay gen after relay_on_time. GEN LED will not light up! Need to use additional LED in series with GEN relay.
	uint8_t delay_switch_to_line;	// *0.1 sec
	uint8_t repeat_switch_to_line_times; // how many times
	uint8_t repeat_switch_to_line;	// after *0.1 sec
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;
struct _EEPROM eeprom;

ISR(INT0_vect, ISR_NAKED)
{
	if(ON_GEN_PORT & ON_GEN_PIN) {
		gen_active = 1;
	}
	reti();
}

ISR(PCINT0_vect, ISR_NAKED) 
{
	if(ON_LINE_PORT & ON_LINE_PIN) {
		line_pin = 1;
//		RELAY_GEN_ON;
	} else {
		line_pin = 0;
		timer_10ms = 6; // *0.001 s
//		RELAY_GEN_OFF;
	}
	reti();
}

ISR(TIM0_OVF_vect) // 0.001 sec
{
	if(++timer_10ms == 10) { // 0.01 sec
		timer_10ms = 0;
		if(!line_pin) {
			LED_LINE_OFF;
			if(doing == WD_NONE) {
				if(pos == WP_LINE) {
					if(gen_active) { // switch to gen
xGEN_ON:				LED_GEN_OFF;
						RELAY_GEN_ON;
						doing = WD_RELAY_GEN;
						timer = eeprom.relay_on_time;
						timer_100ms = 0;
					}
				}
			} else if(doing == WD_RELAY_LINE) {
				RELAY_LINE_OFF;
				if(gen_active) { // switch to gen
					goto xGEN_ON;
				} else {
					repeat_line_cnt = 0;
					doing = WD_NONE;
					pos = WP_UNKNOWN;
				}
			} else if(doing == WD_DELAY_SWITCH_TO_LINE) {
				repeat_line_cnt = 0;
				doing = WD_NONE;
			}
		} else if(line_active != line_pin) LED_LINE_ON;
		line_active = line_pin;
	}
	if(++timer_100ms == 100) { // 0.1 sec
		timer_100ms = 0;
		if(doing == WD_NONE) {
			if(gen_active && !RELAY_GEN_ST) { LED_GEN_ON; } else { LED_GEN_OFF; }
			if(pos == WP_GEN) {
				if(line_active) { // switch to line
					if(!gen_active) {
						LED_GEN_OFF;
						RELAY_GEN_OFF;
						LED_LINE_OFF;
						RELAY_LINE_ON; 
						doing = WD_RELAY_LINE;
						timer = eeprom.relay_on_time;
						timer_100ms = 0;
					} else {
						LED_LINE_ON;
						repeat_line_cnt = eeprom.repeat_switch_to_line_times;
						doing = WD_DELAY_SWITCH_TO_LINE;
						timer = eeprom.delay_switch_to_line;
						timer_100ms = 0;
					}
				}
			} else if(pos == WP_UNKNOWN) {
				if(line_active) {
					RELAY_GEN_OFF;
					RELAY_LINE_ON; 
					LED_LINE_OFF;
					doing = WD_RELAY_LINE;
					timer = eeprom.relay_on_time;
					timer_100ms = 0;
				} else if(gen_active) {
					RELAY_LINE_OFF;
					LED_GEN_OFF;
					RELAY_GEN_ON;
					doing = WD_RELAY_GEN;
					timer = eeprom.relay_on_time;
					timer_100ms = 0;
				}
			}
		} else if(doing == WD_DELAY_SWITCH_TO_LINE) {
			LED_LINE_SW;
		}
		if(timer && --timer == 0) {
			if(doing == WD_RELAY_LINE) {
				RELAY_LINE_OFF;
				LED_LINE_ON;
				if(repeat_line_cnt) {
					timer = eeprom.repeat_switch_to_line;
				} 
				doing = WD_NONE;
				pos = WP_LINE;
			} else if(doing == WD_RELAY_GEN) {
				if(!eeprom.relay_gen_always_on) {
					RELAY_GEN_OFF;
				}
				doing = WD_NONE;
				pos = WP_GEN;
			} else if(doing == WD_DELAY_SWITCH_TO_LINE) {
				repeat_line_cnt = eeprom.repeat_switch_to_line_times;
xLINE_ON:		RELAY_GEN_OFF;
				LED_LINE_OFF;
				RELAY_LINE_ON; 
				doing = WD_RELAY_LINE;
				timer = eeprom.relay_on_time;
				timer_100ms = 0;
			} else { // WD_NONE
				if(repeat_line_cnt) {
					repeat_line_cnt--;
					if(pos == WP_LINE) goto xLINE_ON;
				}
			}
		}
		gen_active = 0;
	}
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler: 1
	DDRB = RELAY_GEN_PIN | RELAY_LINE_PIN | LED_LINE_PIN | LED_GEN_PIN; // Output mode
	//PORTB = ; // Pullup
	PRR = (1<<PRADC); // ADC power off
	MCUCR = (0<<SE) | (0<<SM1) | (0<<SM0) | (0<<ISC01) | (1<<ISC00); // Sleep idle not enable, Any logical change on INT0 generates an interrupt request.
	WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDTIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); // Watchdog - 1 s
	GIMSK = (1<<PCIE) | (1<<INT0); // PCIE: Pin Change Interrupt Enable, INT0: External Interrupt Request 0 Enable
	PCMSK = ON_LINE_PINT;
	//GIFR = (1<<INTF0);
	eeprom_read_block(&eeprom, &EEPROM, sizeof(eeprom));
	if(eeprom.timer_value == 0xFF) {
		eeprom.timer_value = F_CPU / 64 / 1000 - 1; // = 72
		eeprom.relay_on_time = 5;
		eeprom.relay_gen_always_on = 1;
		eeprom.delay_switch_to_line = 100;
		eeprom.repeat_switch_to_line_times = 3;
		eeprom.repeat_switch_to_line = 250;
		//eeprom_write_block(&EEPROM, &eeprom, sizeof(eeprom));
	}
	// Timer 8 bit
	OCR0A = eeprom.timer_value; // OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00); // Timer0 prescaler: 64
	TIMSK0 = (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out

	sei();
    while(1)
	{
		//sleep_cpu();	// power down
		wdt_reset();
		asm volatile("nop");
    }
}

/*
void Delay1ms(uint16_t ms)
{
	while(ms-- > 0) {
		_delay_ms(1);
		wdt_reset();
	}
}

void Delay1s(uint16_t s)
{
	if(s == 0xFFFF) s = 600;
	while(s-- > 0) {
		Delay1ms(1000);
	}
}
*/


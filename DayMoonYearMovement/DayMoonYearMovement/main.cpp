/*
 * DayMoonYearMovement.cpp
 *
 * Created: 2/3/2021 12:04:26 AM
 * Author : passp
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define F_CPU 1000000
#include <util/delay.h>
#include <util/delay_basic.h>

#define SETBIT(x,b) ((x) |= _BV(b))
#define CLRBIT(x,b) ((x) &= ~_BV(b))
#define GETBIT(x,b) ((x) &= _BV(b))


// Connections to the RTC (Epson RX8900)

#define RX8900_SCL_PORT PORTB
#define RX8900_SCL_DDR	DDRB
#define RX8900_SCL_BIT	3

#define RX8900_SDA_PORT PORTB
#define RX8900_SDA_DDR	DDRB
#define RX8900_SDA_BIT	1

#define RX8900_INT_PORT PORTB
#define RX8900_INT_DDR	DDRB
#define RX8900_INT_BIT	2
#define RX8900_INT_INT	PCINT2

#define SDA_PORT	RX8900_SDA_PORT
#define SDA_PIN		RX8900_SDA_BIT

#define SCL_PORT	RX8900_SCL_PORT
#define SCL_PIN		RX8900_SCL_BIT

#define	I2C_PULLUP		1					// Use internal pullups for the TWI lines
#define I2C_SLOWMODE	1					// No rush- we are using pullups so give time for them to pull up. 

#include "SoftI2C.h"

// Connections to the movement windings (Lavet stepper motor)

#define MOVEMENT_A_PORT PORTB
#define MOVEMENT_A_DDR	DDRB
#define MOVEMENT_A_BIT	4

#define MOVEMENT_B_PORT PORTB
#define MOVEMENT_B_DDR	DDRB
#define MOVEMENT_B_BIT	0

// Display LED anode

#define LED_PORT	PORTB
#define LED_DDR		DDRB
#define LED_BIT		0

void movement_mid_h() {
	SETBIT(MOVEMENT_A_PORT , MOVEMENT_A_BIT);		// Pull-up on
	SETBIT(MOVEMENT_A_DDR  , MOVEMENT_A_BIT);		// Full drive on	
}

void movement_mid_l() {
	SETBIT(MOVEMENT_A_DDR  , MOVEMENT_A_BIT);		// Full drive on	
}

// Turn on the pin that drives the movement Lavet motor.
// Phase is 0 or 1 for drive pin low or high 
// Assumes DDR is off and PORT is low (how movement_mid_off() leaves it)

void movement_mid_on( uint8_t phase ) {
	
	if (phase) {
		SETBIT(MOVEMENT_A_PORT , MOVEMENT_A_BIT);		// Pull-up on
	}
	
	SETBIT(MOVEMENT_A_DDR  , MOVEMENT_A_BIT);		// Full drive high or low depending on PORT as set by phase
		
}

void movement_mid_off() {
	CLRBIT(MOVEMENT_A_DDR  , MOVEMENT_A_BIT);		// turn off drive
	CLRBIT(MOVEMENT_A_PORT , MOVEMENT_A_BIT);		// turn off Pull-up 
}


void movement_activate() {
	SETBIT( MOVEMENT_A_DDR , MOVEMENT_A_BIT);
	SETBIT( MOVEMENT_B_DDR , MOVEMENT_B_BIT);
}

void movement_phase_1_on(){
	SETBIT( MOVEMENT_A_PORT , MOVEMENT_A_BIT );
}

void movement_phase_1_off(){
	CLRBIT( MOVEMENT_A_PORT , MOVEMENT_A_BIT );
}


void movement_phase_2_on(){
	SETBIT( MOVEMENT_B_PORT , MOVEMENT_B_BIT );	
}

void movement_phase_2_off(){
	CLRBIT( MOVEMENT_B_PORT , MOVEMENT_B_BIT );
}


void movement_idle() {
	CLRBIT( MOVEMENT_A_DDR , MOVEMENT_A_BIT);
	CLRBIT( MOVEMENT_B_DDR , MOVEMENT_B_BIT);
}	



// ****** RX8900 stuff
// I would love to break this out into its own file, but it needs the `define`s for the TWI pins
// to be able to use the soft TWI header file, so we would have to put this defines somewhere else
// and they gets too complicated for this very simple project. 


#define RX8900_TWI_ADDRESS_W  0x64	// TWI Write address - RX8900 8.9.5 (datasheet page 29)
#define RX8900_TWI_ADDRESS_R  0x65	// TWI Read address  - RX8900 8.9.5 (datasheet page 29)


#define RX8900_EXTENTION_REG   0x0d
#define RX8900_FLAG_REG        0x0e
#define RX8900_CONTROL_REG     0x0f
#define RX8900_BACKUP_REG      0x18

#define RX8900_TC0_REG			0x0b	// Fixed-cycle Timer counter 0
#define RX8900_TC1_REG			0x0c	// Fixed-cycle Timer counter 1



#define RX8900_BACKUP_VDETOFF_bm   (1<<3)      // Setting to 1 disables the low voltage detect
#define RX8900_BACKUP_SWOFF_bm     (1<<4)      // Setting to 1 opens the MOS switch


// Set FOUT on the RX8900 to 1Khz (default 32Khz)


void rx8900_fout_1KHz_x(void) {

	uint8_t reg = 0b00000100;
	i2c_write(RX8900_EXTENTION_REG);
	i2c_write(reg);
	i2c_stop();

}

void rx8900_reg_set( uint8_t reg , uint8_t v ) {

	i2c_start(RX8900_TWI_ADDRESS_W);		// Write
	i2c_write(reg);
	i2c_write(v);
	i2c_stop();

	
}

void rx8900_ex_reg_set( uint8_t v ) {
	
	// EXTENTION register
	// TEST  0 = Normal
	// WADA  0 = Week alarm (rather than day) (we don't care)
	// USEL  1 = Minute update (rather than second) (we don't use, so pick the slower one?)
	// TE    0 = Stops the fixed cycle interrupt (we don't use)
	// FSEL 10 = 1 Hz FOUT
	// TSEL 11 = 1 minute count down interrupt (we don't use, so pick the slowest one?)

	rx8900_reg_set(RX8900_EXTENTION_REG,v);

}

// Set FOUT on the RX8900 to 1Hz (default 32Khz)
// Note that FOE must also be high for output

void rx8900_fout_1Hz(void) {

	const uint8_t ex_reg = 0b00101011;
	rx8900_ex_reg_set(ex_reg);

}

// Set FOUT on the RX8900 to 1Hz (default 32Khz)
// Note that FOE must also be high for output

void rx8900_fout_32KHz(void) {

	const uint8_t ex_reg = 0b00100011;
	rx8900_ex_reg_set(ex_reg);

}

void rx8900_fixed_timer_go() {
	
	rx8900_reg_set( RX8900_TC0_REG , 2 );		// Low byte of count value
	rx8900_reg_set( RX8900_TC1_REG , 0 );		// High nibble of count value
	
	rx8900_reg_set( RX8900_CONTROL_REG , 0b01010000 );	// Enable /INT out on fixed timer, 2 second temp comp
	rx8900_reg_set( RX8900_EXTENTION_REG , 0b00111010); // Enable fixed timer, 1-second base clock

}


// Set FOUT on the RX8900 to 1Hz (default 32Khz)
// Note that FOE must also be high for output

void rx8900_set_int(void) {

	const uint8_t ex_reg = 0b00100011;
	rx8900_ex_reg_set(ex_reg);

}



void rx8900_init() {
	
	i2c_init();
}

// Called anytime the RX8900 has seen low voltage (VLF)
// Sets all the control registers to known values
// Clears the flags, so make sure you save any that you need before calling
// Does not reset the time counter

void rx8900_setup(void) {

	// rx8900_fout_32KHz();
	rx8900_fixed_timer_go();

	// Next the FLAGS register
	
	// Importantly clears the low voltage detect flags to zero
	// also clears a bunch of alarm flags we don't use
	
	const uint8_t flag_reg = 0b00000000;

//	USI_TWI_Write_Data( RX8900_TWI_ADDRESS , RX8900_FLAG_REG , &flag_reg , 1 );

	// Note here we reset. which is OK because to get here the low voltage detect had to be set.

	const uint8_t contrl_reg = 0b01000000;     // 2s temp comp, no timers or interrupts or alarms, NO RESET

//	USI_TWI_Write_Data( RX8900_TWI_ADDRESS , RX8900_CONTROL_REG, &contrl_reg, 1 );
	
	// Default with MOS switch closed, sampled 2ms every 1s
	
	const uint8_t backup_reg= 0;

	// USI_TWI_Write_Data( RX8900_TWI_ADDRESS , RX8900_BACKUP_REG , &backup_reg , 1 );
	
}


// We only need interrupts to wake us, so return from the ISR as quickly as possible. 

// Called when watchdog expires
EMPTY_INTERRUPT( WDT_vect);

// Called on pin change interrupt
EMPTY_INTERRUPT( PCINT0_vect );

// Called on INT0 pin interrupt
EMPTY_INTERRUPT( INT0_vect );


void sleep1s() {
	WDTCR = _BV(WDIE) | _BV( WDE ) | _BV( WDP2 ) | _BV( WDP1 );		// 1s, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT	
}


void sleep16ms() {
	WDTCR = _BV(WDIE) | _BV( WDE ) ;		// 16ms, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT
}

void sleep32ms() {
	WDTCR = _BV(WDIE) | _BV( WDE ) |  _BV( WDP0 );		// 32ms, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT
}


// Sleep until next interrupt, which will be caused by the /INT line being pulled low by the RX8900
// when the fixed timer expires. Note that we enable the pull up before we sleep and then disable it 
// when we wake to save power that would be wasted while the /INT is pulling low against it. This depends on the
// fact that the RX8900 automatically lets go of the /INT line ~7ms after pulling low, so since our stepping
// routing takes ~50ms we know that the /INT will be reset by the time we go back to sleep. 

void sleepUntilISR() {
	SETBIT( RX8900_INT_PORT ,RX8900_INT_BIT );	// Enable pull up
	while ( !GETBIT( RX8900_INT_PORT ,RX8900_INT_BIT  ) );
	sleep_cpu();	
	CLRBIT( RX8900_INT_PORT ,RX8900_INT_BIT );	// Disable pull up	
}

void setup() {
	
	
	// Enable pull-up on /INT pin. This is open-drain output on the RX8900 so we need to pull it up with the ATTINY.	
	SETBIT( RX8900_INT_PORT , RX8900_INT_BIT );		
		
	// Enable interrupt on pin change to wake us every time the fixed-cycle timer pulls /INT low.
	SETBIT( GIMSK  , PCIE );				//When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is enabled.
	
	// Set up to deep sleep between interrupts
	sleep_enable();							// How to do once to enable sleep
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// Lowest power, can wake on interrupts (pin change, WDT)
	sei();									// Allow interrupts (so we can wake on them)
			
	// Activate the movement pins
	//movement_activate();
	//SETBIT( DIDR0 , ADC2D );		// Turn off digital input buffer on the pin connected to the movement. This pin is often floating at Vcc midpoint, so this should keep the buffer from bouncing around. 

	SETBIT( PORTB , 0 );			// Pull up the currently unused pin so it does not float around. 

	// Activate the RTC pins
	rx8900_init();
	// Set RTC to send us A periodic interrupt on the /INT pin
	rx8900_setup();
	
}

int main(void)
{

	setup();
	
	const int on_time_ms = 2;
	const int off_time_ms = 1;
	
	const int pulse_count = 20;
	
	//_delay_ms(1000);


    while (1) 
    {
		/*		
		for( int i=0; i<pulse_count; i++) {
			movement_phase_1_on();
			_delay_ms(on_time_ms);
			movement_phase_1_off();
			_delay_ms(off_time_ms);
		}
		
		//sleep1s();
		sleepUntilISR();

		for( int i=0; i<pulse_count; i++) {
			movement_phase_2_on();
			_delay_ms(on_time_ms);
			movement_phase_2_off();
			_delay_ms(off_time_ms);			
		}
		
		//sleep1s();
		
		sleepUntilISR();
		*/
		
		/*
		movement_phase_1_on();
		_delay_ms(20);
		movement_phase_1_off();
		
		//sleep1s();
		sleepUntilISR();

		movement_phase_2_on();
		_delay_ms(20);
		movement_phase_2_off();
		//sleep1s();
		
		sleepUntilISR();
		*/
		

		//for( uint8_t phase : {0,1} ) {
		for(uint8_t phase=0; phase<2;phase++) {

			// Disable /INT interrupt here to avoid spurious interrupt when /INT rises from waking us from WDT sleep.
			CLRBIT( PCMSK , RX8900_INT_INT );		//Disable interrupt on /INT pin change. This will prevent us from waking from the WDT delay sleep when this pin floats after the RTC stops pulling it low. 		
			SETBIT( DIDR0 , ADC1D );				// Turn off digital input buffer on the pin connected to /INT so will not waste power when it floats when RTC stops pulling it low.
			CLRBIT( RX8900_INT_PORT , RX8900_INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.	
			
			movement_mid_on(phase);
			sleep16ms();
			SETBIT( RX8900_INT_PORT , RX8900_INT_BIT );		// OK, /INT should be reset by the time we get here, so turn on the pull up again. This will give it time to pull the voltage back up by the time we are ready to sleep again. 
			sleep32ms();		
			movement_mid_off();
			
			CLRBIT( DIDR0 , ADC1D );				// Enable digital input buffer on the pin connected to /INT so we will see when RTC pulls it low
			SETBIT( PCMSK , RX8900_INT_INT );	// Enable interrupt on /INT pin change to wake us when RTC says we are ready for next tick. 

			//sleep_cpu();
			//sei();
			//sleepUntilISR();
			//sei();
			//sleep_cpu();			
			//_delay_ms(100);
		}
				
    }
}


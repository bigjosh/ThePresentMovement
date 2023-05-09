/*
 * DayMoonYearMovement.cpp
 *
 * Created: 2/3/2021 12:04:26 AM
 * Author : josh
 */ 


#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>		// _BV()

// From the factory, the CLKDIV8 fuse is set, to we are only running at 1Mhz
// This will switch us to full speed 8Mhz, which is fine because we know we will have >3V from the 2xAA batteries. 

// Power usage is non-linear to speed, so going faster should use less power 

// 8Mhz - you need at least ~2.6V to do this

void fullSpeedClock() {
	CLKPR = (1<<CLKPCE); // Prescaler enable
	CLKPR = 0; // Clock division factor 1 (8Mhz)
}

// 4Mhz - this works all voltages (min 1.8V)

void div2Clock() {
	CLKPR = (1<<CLKPCE); // Prescaler enable
	CLKPR = 1; // Clock division factor 2 (4Mhz)
}

// 1Mhz

void div8Clock() {
	CLKPR = (1<<CLKPCE); // Prescaler enable
	CLKPR = (1<<CLKPS1 || 1<<CLKPS0); // Clock division factor 8 (1Mhz)
}


#define F_CPU 4000000		// Assumes fullSpeedClock() above has been called. This must be defined before the inlcude of SoftI2C.
#include <util/delay.h>
#include <util/delay_basic.h>

#define SETBIT(x,b) ((x) |= _BV(b))
#define CLRBIT(x,b) ((x) &= ~_BV(b))
#define GETBIT(x,b) ((x) & _BV(b))


// *** Hardware connections

// Connections to the RTC (Epson RX8900)

#define RX8900_SCL_PORT PORTB
#define RX8900_SCL_DDR	DDRB
#define RX8900_SCL_BIT	3

#define RX8900_SDA_PORT PORTB
#define RX8900_SDA_DDR	DDRB
#define RX8900_SDA_BIT	1

#define RX8900INT_PORT		PORTB
#define RX8900INT_PIN		PINB
#define RX8900INT_DDR		DDRB
#define RX8900INT_BIT		2
#define RX8900INT_PCMSK	PCINT2

// Define these for the i2c code to see

#define SDA_PORT	RX8900_SDA_PORT
#define SDA_PIN		RX8900_SDA_BIT

#define SCL_PORT	RX8900_SCL_PORT
#define SCL_PIN		RX8900_SCL_BIT

#define	I2C_PULLUP		1					// Use internal pullups for the TWI lines
#define I2C_SLOWMODE	1					// No rush- we are using pullups so give time for them to pull up. 

#define I2C_CPUFREQ 20000000				// Make the I2C think we are running at 20Mhz so it will slow down. 

// We use this not so great i2c code since it does not really matter - we only use it one time on power up and never touch it again
// so not worth rewriting. 

#include "SoftI2C.h"

// Connections to the movement windings (Lavet stepper motor)
// Note that this pin is connected to one end of the motor windings and the other end of the
// windings is connected to the midpoint between the two 1.5V batteries, so driving this pin high
// puts +1.5V on the winding and driving it low puts -1.5V across them. 

// TODO: Put the movement on pin AN0 or AN1 so we can disable the digital input buffer to 
// possibly save power when pin is floating midpoint. 

#define MOVEMENT_PORT	PORTB
#define MOVEMENT_DDR	DDRB
#define MOVEMENT_BIT	4
#define MOVEMENT_DIDR	DIDR0	// Digital input disable register 
#define MOVEMENT_DIDB	ADC2D	// Bit to set in DIDR to turn of the digital input on this pin 


// Push button

#define BUTTON_PORT		PORTB
#define BUTTON_DDR		DDRB
#define BUTTON_PIN		PINB
#define BUTTON_BIT		0
#define BUTTON_PCMSK	PCINT0			// Set this bit in PCMSK to enable pin change interrupt
#define BUTTON_VECT		PCINT0_vect		// Vector called on pin change interrupt


void disableButton() {

	CLRBIT( PCMSK , BUTTON_PCMSK );	   // Disable change interrupt on this pin. 
	CLRBIT( BUTTON_PORT , BUTTON_BIT); // Disable pull-up
	SETBIT( BUTTON_DDR , BUTTON_BIT ); // Drive low to keep from floating. Pushing button will have no effect and waste no power since it is just connecting pin to ground.
	
}

// Returns true if button is pressed

uint8_t isButtonDown() {
	return !GETBIT( BUTTON_PIN ,  BUTTON_BIT );			// NOTed because pushing button grounds pin
	
}


void movement_mid_h() {
	SETBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// Pull-up on
	SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive on	
}

void movement_mid_l() {
	SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive on	
}

// Turn on the pin that drives the movement Lavet motor.
// Phase is 0 or 1 for drive pin low or high 
// Assumes DDR is off and PORT is low (how movement_mid_off() leaves it)

void movement_mid_on( uint8_t phase ) {
	
	if (phase) {
		SETBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// Pull-up on
	}
	
	SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive high or low depending on PORT as set by phase
		
}

void movement_mid_off() {
	CLRBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// turn off drive
	CLRBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// turn off Pull-up if on.
}


// ****** RX8900 stuff
// I would love to break this out into its own file, but it needs the `define`s for the TWI pins
// to be able to use the soft TWI header file, so we would have to put this defines somewhere else
// and they gets too complicated for this very simple project. 


#define RX8900_TWI_ADDRESS_W  0x64	// TWI Write address - RX8900 8.9.5 (datasheet page 29)
#define RX8900_TWI_ADDRESS_R  0x65	// TWI Read address  - RX8900 8.9.5 (datasheet page 29)


#define RX8900_EXTENTION_REG   0x0d


//TE ( Timer Enable ) bit
//This bit controls the start/stop setting for the fixed-cycle timer interrupt function.
//Writing a "1" to this bit specifies starting of the fixed-cycle timer interrupt function (a countdown starts from a
//preset value).
//Writing a "0" to this bit specifies stopping of the fixed-cycle timer interrupt function

#define RX8900_EXTENTION_REG_TE_MASK   _BV(4)		

//TSEL0,1 ( Timer Select 0, 1 ) bits
//The combination of these two bits is used to set the countdown period (source clock) for the fixed-cycle timer
//interrupt function (four settings can be made).
													
#define RX8900_EXTENTION_REG_TSEL1_MASK   _BV(1)		//TE ( Timer Enable ) bit													
#define RX8900_EXTENTION_REG_TSEL0_MASK   _BV(0)		//TE ( Timer Enable ) bit

// These masks control if the fixed cycle timer values in TC0 and TC1 refer to seconds or minutes

#define RX8900_EXTENTION_REG_TSEL_SEC_MASK	 ( RX8900_EXTENTION_REG_TSEL1_MASK )										// "Second" update / Once per second
#define RX8900_EXTENTION_REG_TSEL_MIN_MASK	 ( RX8900_EXTENTION_REG_TSEL1_MASK |  RX8900_EXTENTION_REG_TSEL0_MASK )		// "Minute" update / Once per minute
#define RX8900_EXTENTION_REG_TSEL_64H_MASK	 ( RX8900_EXTENTION_REG_TSEL0_MASK )										// "64Hz" update / Once per 15.625ms


//FSEL0,1 ( Timer Select 0, 1 ) bits
//The combination of these two bits is used to set the FOUT period 
//interrupt function (four settings can be made).

#define RX8900_EXTENTION_REG_FSEL1_MASK   _BV(3)		// FOUT sel bit
#define RX8900_EXTENTION_REG_FSEL0_MASK   _BV(2)		// FOUT sel bit


// These masks control if the FOUT frequency based on values in FSEL0 and FSEL

#define RX8900_EXTENTION_REG_FSEL_32768_MASK	 ( 0x00 )
#define RX8900_EXTENTION_REG_FSEL_1024_MASK	 ( RX8900_EXTENTION_REG_FSEL0_MASK )
#define RX8900_EXTENTION_REG_FSEL_1_MASK	 ( RX8900_EXTENTION_REG_FSEL1_MASK )


#define RX8900_FLAG_REG        0x0e			

#define RX8900_CONTROL_REG     0x0f

#define RX8900_CONTROL_REG_CSEL1     _BV(7)
#define RX8900_CONTROL_REG_CSEL0     _BV(6)

//CSEL0,1 ( Compensation interval Select 0, 1 ) bits
//The combination of these two bits is used to set the temperature compensation interval.

#define RX8900_CONTROL_REG_TEMPCOMP_2S_MASK     (RX8900_CONTROL_REG_CSEL0)		// (0,1) 2.0s Default
#define RX8900_CONTROL_REG_TIE_MASK				_BV(4)							// TIE ( Timer Interrupt Enable ) bit
																				// When a fixed-cycle timer interrupt event occurs (when the TF bit value changes from "0" to "1"), this bit's value
																				// specifies if an interrupt signal is generated (/INT status changes from Hi-Z to low) or is not generated (/INT status
																				// remains Hi-Z).
																				// When a "1"?is written to this bit, an interrupt signal is generated (/INT status changes from Hi-Z to low) when an
																				// interrupt event is generated.
																				// When a "0"?is written to this bit, no interrupt signal is generated when an interrupt event occurs.

#define RX8900_BACKUP_REG      0x18

#define RX8900_BACKUP_REG_VDETOFF_MASK      _BV(3)				// VDETOFF bit (Voltage Detector OFF). This bit controls the voltage detection circuit of the main power supply VDD.
#define RX8900_BACKUP_REG_SWOFF_MASK		_BV(2)				// This bit controls the internal P-MOS switch for preventing back flow.

#define RX8900_BACKUP_REG_DISABLED_VALUE    (RX8900_BACKUP_REG_VDETOFF_MASK)		// This seems to be the recommended config for no backup. Not clear why we leave the switch ON, but that shown in the diagram. 

// How many counts in each fixed cycle timer interrupt. Only 12 bits long. 

#define RX8900_TC0_REG			0x0b	// Fixed-cycle Timer counter 0
#define RX8900_TC1_REG			0x0c	// Fixed-cycle Timer counter 1


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


void rx8900_fixed_timer_stop() {

	rx8900_reg_set( RX8900_EXTENTION_REG , RX8900_EXTENTION_REG_FSEL_1_MASK ); // Disable fixed timer so when we start, it starts at the top of the period
	
}

// All of these EXTTENTION_REG functions set the FOUT to 1Hz which is the lowest setting to use less power since we do not even have FOUT connected.  

// Start fixed timer with period minutes whatever last set in set_fixed_time_count(). Maybe not nessisary since we tie FOE down to disable FOUT? 

void rx8900_fixed_timer_start_mins() {

	rx8900_reg_set( RX8900_EXTENTION_REG ,  RX8900_EXTENTION_REG_TE_MASK | RX8900_EXTENTION_REG_TSEL_MIN_MASK | RX8900_EXTENTION_REG_FSEL_1_MASK ); // Enable fixed timer, 1-minute base clock
	
}

// Start fixed timer with period seconds whatever last set in set_fixed_time_count()

void rx8900_fixed_timer_start_secs() {

	rx8900_reg_set( RX8900_EXTENTION_REG , RX8900_EXTENTION_REG_TE_MASK | RX8900_EXTENTION_REG_TSEL_SEC_MASK | RX8900_EXTENTION_REG_FSEL_1_MASK); // Enable fixed timer, 1-second base clock
	
}


void rx8900_fixed_timer_start_64h() {

	rx8900_reg_set( RX8900_EXTENTION_REG , RX8900_EXTENTION_REG_TE_MASK | RX8900_EXTENTION_REG_TSEL_64H_MASK | RX8900_EXTENTION_REG_FSEL_1_MASK); // Enable fixed timer, 1-second base clock
	
}


// Set the RX8900 counts between INT events (MAX = 4195)
// A count can be a second or a minute or 1/64th of sec (and others we don't care about) depending on TSEL bits when timer is started.
// Count starts when TE bit enabled

// Note that c only has 12 significant bits so max value is 4095

void rx8900_fixed_timer_set_count( const uint16_t c ) {	
	rx8900_reg_set( RX8900_TC0_REG , c & 0xff );		// Low byte of count value (in seconds)
	rx8900_reg_set( RX8900_TC1_REG , c / 0xff );		// High nibble of count value
}

// When a "1" is written to the TE bit, the fixed-cycle timer countdown starts from the preset value.

void rx8900_fixed_timer_start_seconds( unsigned s ) {

	rx8900_fixed_timer_set_count(s);
	
	rx8900_fixed_timer_start_secs();

}

// When a "1" is written to the TE bit, the fixed-cycle timer countdown starts from the preset value.


void rx8900_fixed_timer_start_minutes( unsigned s ) {
	
	rx8900_fixed_timer_set_count(s);
	
	rx8900_fixed_timer_start_mins();

}

void rx8900_fixed_timer_start_64h( unsigned s ) {
	
	rx8900_fixed_timer_set_count(s);
	
	rx8900_fixed_timer_start_64h();

}


void rx8900_init() {
	
	i2c_init();
}

// Turn off the /INT output

void rx8900_disable_int() {

	// Stop any running fixed timer. Set FOUT to 1Hz.	
	rx8900_reg_set( RX8900_EXTENTION_REG , RX8900_EXTENTION_REG_FSEL_1_MASK );
	
}

// Called anytime the RX8900 has seen low voltage (VLF)
// Sets all the control registers to known values
// Clears the flags, so make sure you save any that you need before calling
// stops any running fixed cycle counter
// Does not reset the time counter

void rx8900_setup(void) {

	// 2 second temp comp, enable the fixed timer interrupt on /INT (but it will not fire until fixed timer is enabled)
	
	rx8900_reg_set( RX8900_CONTROL_REG , RX8900_CONTROL_REG_TEMPCOMP_2S_MASK | RX8900_CONTROL_REG_TIE_MASK );	
	
	// MOS switch ON, Voltage detector OFF. We have main battery wired to both Vcc and Bat so no backup detecting wanted. 
	
	rx8900_reg_set( RX8900_BACKUP_REG , RX8900_BACKUP_REG_DISABLED_VALUE  );
	
}


// We only need interrupts to wake us, so return from the ISR as quickly as possible. 

// Called when watchdog expires
EMPTY_INTERRUPT( WDT_vect);

// Called on INT0 pin interrupt
EMPTY_INTERRUPT( INT0_vect );

// TODO: We can save some power by moving the button to the INT0 vector so we don't have to test for it in the main /INT handler.

// Called on pin change interrupt (from the RTC or button depending on when pins enabled)
EMPTY_INTERRUPT( PCINT0_vect );


// Uses watchdog for power savings, so interrupts must be on 
// Assumes no other interrupt sources can wake us
void sleep1s() {
	WDTCR = _BV(WDIE) | _BV( WDE ) | _BV( WDP2 ) | _BV( WDP1 );		// 1s, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT	
}

// Uses watchdog for power savings, so interrupts must be on
// Assumes no other interrupt sources can wake us
inline void sleep16ms() {
	WDTCR = _BV(WDIE) | _BV( WDE ) ;		// 16ms, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT
}

// Uses watchdog for power savings, so interrupts must be on
// Assumes no other interrupt sources can wake us
inline void sleep32ms() {
	WDTCR = _BV(WDIE) | _BV( WDE ) |  _BV( WDP0 );		// 32ms, interrupt enable
	sleep_cpu();
	WDTCR = _BV(WDCE) | _BV(WDE);	// Allow change the WDE bit
	WDTCR = 0;						// Disable WDT
}


// Spin at full speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)
// Uses watchdog for power savings, so interrupts must be on
// Assumes no other interrupt sources can wake us

uint8_t spin( uint16_t ticks ) {
	
	uint8_t phase = 0 ;
	
	while (ticks--)  {
		
		movement_mid_on(phase);
		//_delay_ms(50);
					
		sleep16ms();
		sleep32ms();
		movement_mid_off();
		sleep16ms();
		sleep32ms();
		phase = !phase;
	}
	
	return phase;

}

void twitch_noINT( uint16_t ticks ) {
	
	
	while (ticks>0)  {
		
		movement_mid_on(0);
		_delay_ms(50);
		
		movement_mid_off();
		_delay_ms(50);
		
	
		ticks--;
	}
	
}

// Spin at full speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)
// This version does not use any interrupts, so since it does not sleep between ticks it uses more power. 

uint8_t spin_noINT_with_warmup( uint16_t ticks ) {
	
	uint8_t phase = 0 ;
	
	uint8_t warmup_ticks=4;
		
	while (warmup_ticks>0 && ticks>0)  {
		
		phase = phase ? 0 : 1;
				
		movement_mid_on(phase);
		_delay_ms(50);
		
		movement_mid_off();
		_delay_ms(50);
				
		warmup_ticks--;
		ticks--;
	}
	
	
	while (ticks>0)  {
		
		phase = phase ? 0 : 1;
		
		
		movement_mid_on(phase);
		_delay_ms(25);
		
		movement_mid_off();
		_delay_ms(25);
				
		ticks--;
	}
	
	return phase;

}

// Spin at full speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)
// This version does not use any interrupts, so since it does not sleep between ticks it uses more power.

uint8_t spin_noINT( uint8_t phase , uint16_t ticks ) {
			
	while (ticks>0)  {
		
		phase = phase ? 0 : 1;
				
		movement_mid_on(phase);
		_delay_ms(30);
		
		movement_mid_off();
		_delay_ms(30);
		
		ticks--;
	}
	
	return phase;

}



// Spin at half speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)
// This version does not use any interrupts, so since it does not sleep between ticks it uses more power.

uint8_t spinhalf_noINT( ) {
	
	
	uint8_t phase = 0 ;
	
	while (1)  {
		
		movement_mid_on(phase);
		_delay_ms(100);
		
		movement_mid_off();
		_delay_ms(100);
		
		phase = phase ? 0 : 1;
	}
	
	return phase;

}

// Run the clock normally, moving the hand one step per tick. Runs forever. 
// The tick is generated by the RX8900 ~INT pin, and the frequency that this
// pin generates a tick is controlled by the Timer Counter (TC) register and TSEL bits.
// Assumes interrupts on, global pin change enabled, /INT pin pulled up, RX8900 programmed to generate /INT signal, digital input buffer on /INT pin enabled, and /INT pin mask enabled. 
// Assumes the stepper motor last had movement_mid_on(0) on. 
// Never returns.
// Note that we have some code in here that is repeated form other places, but it is so that we can be as efficient as possible since this loop runs for the life of the clock. 

void normalStepMode() {
		 
	while (1) {			// TODO: Unroll this loop for slight efficiency increase
		
		// Wait for next tick from the RTC
				
		sleep_cpu();			// Wait for /INT to wake us
				
		CLRBIT( RX8900INT_PORT , RX8900INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.
													// Do this as quickly as possible after waking to save power wasted though the pull-up.
													// TODO: worth it to do this in the ISR since it would be called slightly sooner?

		// Disable /INT interrupt here to avoid spurious interrupt when /INT rises when the RX8900 auto-resets it in 7ms.
		CLRBIT( PCMSK , RX8900INT_PCMSK );		// Disable interrupt on /INT pin change. This will prevent us from waking from the WDT delay sleep when this pin floats after the RTC stops pulling it low.
												// "Earliest 7.813 ms after the interrupt occurs, the /INT status is automatically cleared (/INT status changes from low-level to Hi-Z)."
		
		SETBIT( DIDR0 , ADC1D );				// Turn off digital input buffer on the pin connected to /INT so will not waste power when it floats when RTC stops pulling it low.


		// Actually move the stepper motor - PHASE 1, which is the midpoint pin driven high 
		SETBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// Pull-up on
		SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive high or low depending on PORT as set by phase
		
		sleep16ms();
		SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// OK, /INT should be reset by the time we get here, so turn on the pull up again. This will give it time to pull the voltage back up by the time we are ready to sleep again.
		
				
		sleep32ms();
		
		// Now turn the motor off by making the midpoint pin float
						
		CLRBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// Now drive pin low to quench flyback voltage
		CLRBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// turn off drive
	
		CLRBIT( DIDR0 , ADC1D );				// Enable digital input buffer on the pin connected to /INT so we will see when RTC pulls it low
		SETBIT( PCMSK , RX8900INT_PCMSK );		// Enable interrupt on /INT pin change to wake us when RTC says we are ready for next tick.

		#warning
		//div8Clock();		
		//_delay_us(100/8)		;		// keep processor running to absorb inductive kick?
		//fullSpeedClock();
					
		// This step is done
		// Wait for next tick from the RTC
		
		/*
		_delay_us(100);
		div8Clock();
		_delay_us(100);
		fullSpeedClock();		
		*/

		sleep_cpu();			// Wait for /INT to wake us
		
		CLRBIT( RX8900INT_PORT , RX8900INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.
		// Do this as quickly as possible after waking to save power wasted though the pull-up.
		// TODO: worth it to do this in the ISR?

		// Disable /INT interrupt here to avoid spurious interrupt when /INT rises when the RX8900 auto-resets it in 7ms.
		CLRBIT( PCMSK , RX8900INT_PCMSK );		// Disable interrupt on /INT pin change. This will prevent us from waking from the WDT delay sleep when this pin floats after the RTC stops pulling it low.
		// "Earliest 7.813 ms after the interrupt occurs, the /INT status is automatically cleared (/INT status changes from low-level to Hi-Z)."
		
		SETBIT( DIDR0 , ADC1D );				// Turn off digital input buffer on the pin connected to /INT so will not waste power when it floats when RTC stops pulling it low.

		// Actually move the stepper motor - PHASE 0, which is the midpoint pin driven low
		SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive low since PORT is already LOW
		
		sleep16ms();
		SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// OK, /INT should be reset by the time we get here, so turn on the pull up again. This will give it time to pull the voltage back up by the time we are ready to sleep again.
		
		
		sleep32ms();
		
		// Now turn the motor off by making the midpoint pin float
		// IN this phase we do not need to set the PORT low because we were driving low

		CLRBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// turn off drive
		CLRBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// quench inductive kickback
			
		CLRBIT( DIDR0 , ADC1D );				// Enable digital input buffer on the pin connected to /INT so we will see when RTC pulls it low
		SETBIT( PCMSK , RX8900INT_PCMSK );		// Enable interrupt on /INT pin change to wake us when RTC says we are ready for next tick.			

		// This step is done
					
	}	
	
	__builtin_unreachable();
	
}

// Assumes that no other interrupts (besides the WDT that we enable) will happen during its run. 
// Leaves movement pin floating. 
// This takes ~50ms and we should rest of 50ms after. 
// Safe if called right after /INT interrupt and that only happens once every ~100ms

void step( uint8_t phase ) {
	
	if (phase) {
		// Actually move the stepper motor - PHASE 1, which is the midpoint pin driven high
		SETBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// Pull-up on
	} 
	
	SETBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// Full drive high or low depending on PORT as set by phase
	
	// TODO: Sleep here to save power
	_delay_ms(50);
		
	CLRBIT(MOVEMENT_DDR  , MOVEMENT_BIT);		// turn off drive
	CLRBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// turn off pull-up in phase 1 (does nothing in phase 0, but faster than checking and branching)
	
}

// Low power wait for the button to be pressed. Assumes no other interrupts running.
// Assumes interrupts enabled and PCIE enabled in GIMSK, but no other interrupts on.
// Returns with button interrupt disabled. 

void waitButtonPress() {
	
	// Enable the hardware

	sleep32ms();					   // Give pull-up time to raise voltage on pin to avoid spurious change	
	
		
	while (!isButtonDown());			// Wait for button to be pressed before starting.
	
	disableButton();
	
	sei();
	
	
}

#define TICKS_PER_ROTATION (3600)

#define LUNARMONTH_SECS (2551443UL)			//https://www.justintools.com/unit-conversion/time.php?k1=lunar-months&k2=seconds
#define LUNARMONTH_SECS_PER_TICK ( LUNARMONTH_SECS / TICKS_PER_ROTATION )

#define SOLARDAY_SECS (60UL*60*24)			
#define SOLARDAY_SECS_PER_TICK ( SOLARDAY_SECS/ TICKS_PER_ROTATION )

#define TROPICALYEAR_SECS (31556925UL)
#define TROPICALYEAR_SECS_PER_TICK ( SOLARDAY_SECS/ TICKS_PER_ROTATION )
#define TROPICALYEAR_MINS_PER_TICK ( TROPICALYEAR_SECS_PER_TICK / SECS_PER_MIN )

#define HOUR_SECS	(60*60)
#define HOUR_SECS_PER_TICK ( HOUR_SECS / TICKS_PER_ROTATION )


// Human life is long enough that we have to switch to minutes

#define HUMANLIFE_YEARS (120)
#define SECS_PER_MIN (60)

#define MINS_PER_TROPICALYEAR (TROPICALYEAR_SECS/SECS_PER_MIN)

#define HUMANLIFE_MINS (HUMANLIFE_YEARS*MINS_PER_TROPICALYEAR)
#define HUMANLIFE_MINS_PER_TICK (HUMANLIFE_MINS / TICKS_PER_ROTATION )		// About 17,000 mins per tick. TODO: Add error correction here to account for minute aliasing

int main(void)
{

	// Switch from bootup 1Mhz to half speed 4Mhz (the fastest we can reliably go since the battery voltage will drop below 2.6V which is min for 8Mhz.
	div2Clock();
	
	// If we reset because we are about to reprogram, give the programmer a chance to control the shared data line
	// before we start using it for I2C stuff. 
		
	// First thing we will setup the RTC chip, which will stop it from generating /INT signals if it is currently running.
	// We do this because the /INT line is shared with the CLK programming line, so by turning it off on power up we 
	// give ourselves a chance to reprogram the chip without getting interrupted by that /INT signal. 

	i2c_init();
	_delay_ms(1);		// Give the pullup on data line a chance to pull up so the start bit is not too short. 
	

/*
#define RX8900_SCL_PORT PORTB
#define RX8900_SCL_DDR	DDRB
#define RX8900_SCL_BIT	3

#define RX8900_SDA_PORT PORTB
#define RX8900_SDA_DDR	DDRB
#define RX8900_SDA_BIT	1


	SETBIT( RX8900_SDA_PORT , RX8900_SDA_BIT);
	SETBIT( RX8900_SDA_DDR , RX8900_SDA_BIT);

	SETBIT( RX8900_SCL_PORT , RX8900_SCL_BIT);
	
*/	

	// First we disable the int so in case we are in a programming cycle, it will not mess up the data line. 
	rx8900_disable_int();

	// Now we can wait. The programmer will only release RESET for 50ms, so if we are in a programming cycle
	// then we will never leave this delay. 
	_delay_ms(50);
	
	// If we get to here, then it was a normal power up so we have plenty of time to set the RX8900 up right
	rx8900_setup();	

	
	// *** Program RTC to generate /INT signal period
	// (also will stop any previously running fixed timer)
	
			
	// Lets start with a quick run around the dial to prove we are awake an working
	// This also gives us a chance to reprogram after a power up before the /INT signal starts
	// Remember what phase we ended in.

	uint8_t motor_phase;
	
	motor_phase= spin_noINT( 1 ,60);
	
	//CLRBIT( RX8900_SDA_PORT , RX8900_SDA_BIT);
	//while (1);
	
					
	// *** First set up all pins and pull-ups

	SETBIT( BUTTON_PORT , BUTTON_BIT);				// Enable button pull-up. Button connects pin to ground.
	
	SETBIT( MOVEMENT_DIDR , MOVEMENT_DIDB);	// Turn off digital input on movement pin to save power. We only use this for output, and it is floating at vcc/2 in between steps. 
		
	SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// Enable pull-up on /INT pin. This is open-drain output on the RX8900 so we need to pull it up with the ATTINY.
		
	_delay_ms(10);		// Give the pull-ups a chance to pull the voltages. Don't use the WDT-based waits because they might get interrupted by the above changing pins. 
			
	//  ALL PINS NOW CONFIGURED FOR PULL-UP
	
	// *** Global interrupt preparation
	
	// Note that all ISRs point to an empty handler - we only use them to wake us and resume form where were went to sleep.
	
	// Enable interrupt on pin change.
	// "Any change on any enabled PCINT[5:0] pin will cause an interrupt."
	// Note that individual pins must also be enabled in the mask
	// Used to wake us every time the fixed-cycle timer pulls /INT low or when button is pressed (each must be enabled in PCMSK)
	SETBIT( GIMSK  , PCIE );				// When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is enabled.
	
	sei();			// And finally ready to actually enable interrupts. Still need to enable individual pins in the mask as needed.
	

	// *** Enable interrupts on pins connected to /INT signal from RTC and button (global pin interrupt already enabled above)
	// Note that both pins are on the same vector so the only way to tell what happened is to check the bits
	SETBIT( PCMSK , RX8900INT_PCMSK );	   // Enable change interrupt on /INT pin change from RTC.
	
	// TODO: Is 3/64th of a second good enough for button responsiveness without an interrupt? Lets see!
	//SETBIT( PCMSK , BUTTON_PCMSK );	   // Enable change interrupt on button pin. You must also sei() and enable PCIE in GIMSK to enable all pin change interrupts
			
	// Set the fixed timer period count
	// (we will specify if this count is minutes or seconds when we start the fixed timer

	// Minutes in a year
	// https://www.google.com/search?q=%28%281+years%29%2F%283600%29%29+in+minutes
	//rx8900_fixed_timer_set_count(146);
	
	// One rotation per day per 24 hours = 24 seconds per tick (1 second per tick= 1 hour per rotation)
	// rx8900_fixed_timer_set_count( 24 );
				
	// *** Enable sleeping mode
	
	sleep_enable();							// Do once to enable sleep forever
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// Lowest power, can wake on interrupts (pin change, WDT)		

	
	// Our base time unit `tick` will be 6/64ths of a second. We pick this because...
	// 1. We can precisely set it on our RTC which has a 64hz base clock, and
	// 2. 6/64ths is just about 100ms  which is approx our (current) minimum time per step on the clock movement (we might be able to get this down by driving the motor differently)
	//#warning
	rx8900_fixed_timer_set_count(8);
	
	// The Timer Enable bit will be 0 now from reset(), so this will set TE to 1 and the timer will start at the top of the period. 	
	
	rx8900_fixed_timer_start_64h();	
	

	// Twitch once per sec
	unsigned long c=0;
	while (1) {
		
		sleep_cpu();				// Wait for RTC tick to wake us		
		
		
		if ( !GETBIT(RX8900INT_PIN,RX8900INT_BIT)  ) {

			CLRBIT( RX8900_SDA_PORT , RX8900_SDA_BIT);			
			
			_delay_ms(1);
				
			SETBIT( RX8900_SDA_PORT , RX8900_SDA_BIT);			
						
			c++;
			if ((c&7)==0) {
					step(motor_phase);	
			}
		}
		
	};

	
	
	uint8_t in_learn_mode_flag = 0;			// Are we currently in learn mode where we are waiting for 2nd button press to establish the period?
	unsigned long period_ticks=((64*60)/3);	// The period in 3/64s. Power up with a 1 minute period.
	unsigned long current_ticks=0;			// Current tick counter incremented on each tick.
	uint8_t old_port_bits = 0;				// This chip does not latch pin changes, so we must remember the previous bits so we can tell what changes. Start assuming button is UP. If it is down, then we will ignore until it goes up first.
	
	while (1) {
		
		sleep_cpu();				// Wait for RTC tick to wake us
		
		// static_assert( BUTTON_PIN == RX8900INT_PIN , "Button and RTC pins must be on same PORT" );
		uint8_t new_port_bits = BUTTON_PIN;	// Save which gpio bits got us here. Captures both the button and RTC /INT pins.
		
		// First check for button change
		if ( GETBIT( new_port_bits , BUTTON_BIT) != GETBIT( old_port_bits , BUTTON_BIT)  ) {
			// Button state changed
					
			if (!GETBIT(new_port_bits,BUTTON_BIT)) {			// Button has pull-up, so pin==0 means button pushed down
				// Button newly down 
				
				if (in_learn_mode_flag) {
					// end learn mode!
					period_ticks=current_ticks;		// Remember the learned period
					current_ticks=0;				// Start ticking now
					in_learn_mode_flag=0;			// end learn mode
					
				} else {
					// Start learn mode!
					current_ticks=0;		// Clear counter
					in_learn_mode_flag=1;
				}
															
			}
			
		}
		
		// Next handle the tick
		// Currently the RTC is the only enabled interrupt source, so no need to check if it triggered.
		
		if (in_learn_mode_flag) {
			
			// We are in learning mode, so twittle the hand about once a sec
			// We do not update the phase, so stepping the same phase just moves the hand forward a bit but then it pulls back again. 
			
			if ( (current_ticks & 7) == 0) {		// This is a quick way to do it
				step(motor_phase);			
			}
			
		} else {
			
			if (current_ticks*60>=period_ticks) {			// 60 steps per rotation *  1 rotation per period  * period_ticks per rotation 
				// Time to move the hand 
				
				motor_phase = !motor_phase;		// switch phase so hand moves one step forward
				step( motor_phase );
				current_ticks -= period_ticks;  // Rather than just resetting current_ticks to 0, this will bresenham us to always accumulate and eventually compensate for the error term. 
			}
			
			
		}
		
		current_ticks++;		// increments the tick counter
				
		old_port_bits=new_port_bits;
				
	}

	
	__builtin_unreachable();
		
}
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
// This will switch us to full speed 8Mhz, which is fine becuase we know we will have >3V from the 2xAA batteries. 

void fullSpeedClock() {
	CLKPR = (1<<CLKPCE); // Prescaler enable
	CLKPR = 0; // Clock division factor 1 (8Mhz)
}

void div8Clock() {
	CLKPR = (1<<CLKPCE); // Prescaler enable
	CLKPR = (1<<CLKPS1 || 1<<CLKPS0); // Clock division factor 8 (1Mhz)
}


#define F_CPU 8000000		// Assumes fullSpeedClock() above has been called to get rid of DIV8 that is normally set up power up. This must be defined before the inlcude of SoftI2C.

#include <util/delay.h>
#include <util/delay_basic.h>

#define SETBIT(x,b) ((x) |= _BV(b))
#define CLRBIT(x,b) ((x) &= ~_BV(b))
#define GETBIT(x,b) ((x) &= _BV(b))


// *** Hardware connections

// Connections to the RTC (Epson RX8900)

#define RX8900_SCL_PORT PORTB
#define RX8900_SCL_DDR	DDRB
#define RX8900_SCL_BIT	3

#define RX8900_SDA_PORT PORTB
#define RX8900_SDA_DDR	DDRB
#define RX8900_SDA_BIT	1

#define RX8900INT_PORT		PORTB
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


// Unused pin. Currently unconnected.

#define UNUSEDPIN_PORT	PORTB
#define UNUSEDPIN_DDR	DDRB
#define UNUSEDPIN_BIT	0


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
	CLRBIT(MOVEMENT_PORT , MOVEMENT_BIT);		// turn off Pull-up 
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

// These two masks control if the fixed cycle timer values in TC0 and TC1 refer to seconds or minutes

#define RX8900_EXTENTION_REG_TSEL_SEC_MASK	 ( RX8900_EXTENTION_REG_TSEL1_MASK )										// "Second" update / Once per second
#define RX8900_EXTENTION_REG_TSEL_MIN_MASK	 ( RX8900_EXTENTION_REG_TSEL1_MASK |  RX8900_EXTENTION_REG_TSEL0_MASK )		// "Minute" update / Once per minute


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

	rx8900_reg_set( RX8900_EXTENTION_REG , 0x00 ); // Disable fixed timer so when we start, it starts at the top of the period
	
}

// Start fixed timer with period minutes whatever last set in set_fixed_time_count()

void rx8900_fixed_timer_start_mins() {

	rx8900_reg_set( RX8900_EXTENTION_REG ,  RX8900_EXTENTION_REG_TE_MASK | RX8900_EXTENTION_REG_TSEL_MIN_MASK ); // Enable fixed timer, 1-minute base clock
	
}

// Start fixed timer with period seconds whatever last set in set_fixed_time_count()

void rx8900_fixed_timer_start_secs() {

	rx8900_reg_set( RX8900_EXTENTION_REG , RX8900_EXTENTION_REG_TE_MASK | RX8900_EXTENTION_REG_TSEL_SEC_MASK); // Enable fixed timer, 1-second base clock
	
}


// Set the RX8900 counts between INT events (MAX = 4195)
// A count can be a second or a minute (and others we don't care about) depending on TSEL bits.
// Count starts when TE bit enabled

void rx8900_fixed_timer_set_count( uint16_t c ) {	
	rx8900_reg_set( RX8900_TC0_REG , c & 0xff );		// Low byte of count value (in seconds)
	rx8900_reg_set( RX8900_TC1_REG , c / 0xff );		// High nibble of count value
}


// When a "1" is written to the TE bit, the fixed-cycle timer countdown starts from the preset value.

void rx8900_fixed_timer_start_seconds( unsigned s ) {

	rx8900_fixed_timer_set_count(s);
	
	rx8900_fixed_timer_start_secs();

}

// When a "1" is written to the TE bit, the fixed-cycle timer countdown starts from the preset value.


void rx8900_fixed_timer_set_minutes( unsigned s ) {
	
	rx8900_fixed_timer_set_count(s);
	
	rx8900_fixed_timer_start_mins();

}



void rx8900_init() {
	
	i2c_init();
}

// Called anytime the RX8900 has seen low voltage (VLF)
// Sets all the control registers to known values
// Clears the flags, so make sure you save any that you need before calling
// stops any running fixed cycle counter
// Does not reset the time counter

void rx8900_setup(void) {

	// Stop any running fixed timer
	
	rx8900_fixed_timer_stop();

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


// Spin at full speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)
// This version does not use any interrupts, so since it does not sleep between ticks it uses more power. 

uint8_t spin_noINT( uint16_t ticks ) {
	
	
	
	uint8_t phase = 0 ;
	
	while (ticks--)  {
		
		movement_mid_on(phase);
		_delay_ms(50);
		
		movement_mid_off();
		_delay_ms(50);
		
		phase = phase ? 0 : 1;
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

enum phase_t {A,B};

// Take a single step on the stepper. Phase should alternate between A and B.
// Implemented with a template so that it expands to full code with compiletime PORTs and BITs that can use fast GPIO instructions.

template <phase_t phase>
void normalStep(){
	
	if (phase==A) {
		CLRBIT( RX8900INT_PORT , RX8900INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.
	} else {
		SETBIT( RX8900INT_PORT , RX8900INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.		
	}
	
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
		
		normalStep<A>();
		
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
	
	// Switch from bootup 1Mhz to full speed 8Mhz
	fullSpeedClock();
	
	//while (1);
	// *** First set up all pins and pull-ups
	
	SETBIT( BUTTON_PORT , BUTTON_BIT);				// Enable button pull-up. Button connects pin to ground.
	
	SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// Enable pull-up on /INT pin. This is open-drain output on the RX8900 so we need to pull it up with the ATTINY.
	
	SETBIT( UNUSEDPIN_PORT , UNUSEDPIN_BIT );		// Pull unused pin high. This pin is connected so we don't want it to float and waste power. Is pull-up better than drive-up or drive-low? I don't know.
	
	i2c_init();										// This activates the pull-ups on the i2c SDA and SCL pins. 
	
	_delay_ms(10);		// Give the pull-ups a chance to pull the voltages. Don't use the WDT-based waits because they might get interrupted by the above changing pins. 
	
	//  ALL PINS NOW CONFIGURED FOR PULL-IP
	
	// *** Global interrupt preparation
	
	// Note that all ISRs point to an empty handler - we only use them to wake us and resume form where were went to sleep.
	
	// Enable interrupt on pin change.
	// "Any change on any enabled PCINT[5:0] pin will cause an interrupt."
	// Note that individual pins must also be enabled in the mask
	// Used to wake us every time the fixed-cycle timer pulls /INT low or when button is pressed (each must be enabled in PCMSK)
	SETBIT( GIMSK  , PCIE );				//When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is enabled.
	
	sei();			// And finally ready to actually enable interrupts. Still need to enable individual pins in the mask as needed.

	// Get timer settings ready (will start it later)	
	
	// *** Program RTC to generate /INT signal period
	// (also will stop any previously running fixed timer)
	rx8900_setup();


	// *** Enable interrupt on pin connected to /INT signal from RTC (global pin interrupt already enabled above)
	SETBIT( PCMSK , RX8900INT_PCMSK );	   // Enable change interrupt on /INT pin change from RTC.
	
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
			
	#warning test with 1 second ticks
	rx8900_fixed_timer_set_count(1);		// two second ticks




	// *** Wait for a button press to get us started		
	
	SETBIT( PCMSK , BUTTON_PCMSK );	   // Enable change interrupt on this pin. You must also sei() and enable PCIE in GIMSK to enable all pin change interrupts
	
	sleep_cpu();						// Sleep - we will wake up when button state changes.

	CLRBIT( PCMSK , BUTTON_PCMSK );	   // Disable the interrupt that got us here.

	
	CLRBIT( BUTTON_PORT , BUTTON_BIT );		// Disable the pull-up
	SETBIT( BUTTON_DDR , BUTTON_BIT );		// Drive button pin low so it will not use any power (we never need to check it again)
											// Note we do not leave pull-up on since then would use power as long as button is pressed. 
											
											
	// *** Start out clock from this moment the button is pressed (behavior for now)
	// The Timer Enable bit will be 0 now from reset(), so this will set TE to 1 and the timer will start at the top of the period. 	
	
	rx8900_fixed_timer_start_secs();	
	normalStepMode();
	__builtin_unreachable();											

	
	// rx8900_fixed_timer_start_mins();
	rx8900_fixed_timer_start_secs();

	

	// *** Show we woke	
		
	#define SECS_PER_SCOTTLIFE (1335721787UL)
	#define MINS_PERSCOTTLIFE (SECS_PER_SCOTTLIFE/SECS_PER_MIN)

/*
	
	// The numbers are just too big for even long ints, so use a float. 
	const double HUMANLIFE_SECS =  (1.0 * HUMANLIFE_YEARS * TROPICALYEAR_SECS);
	
	spin( (unsigned) ( (SECS_PER_SCOTTLIFE / HUMANLIFE_SECS ) * 3600.0) );			
	
*/

	// Start the fixed cycle timer (we set up the timer and specified the count above)
	//rx8900_fixed_timer_start_secs();	

	spin(  TICKS_PER_ROTATION / 2  );		// Make 180 deg rotation on button press to show we are working
	
			

	
	//rx8900_fixed_timer_set_seconds( 4 );		
	//rx8900_fixed_timer_set_seconds( SOLARDAY_SECS_PER_TICK );	
	//rx8900_fixed_timer_set_seconds( LUNARMONTH_SECS_PER_TICK );
	//rx8900_fixed_timer_set_minutes( HUMANLIFE_MINS_PER_TICK );
	//rx8900_fixed_timer_set_minutes(  TROPICALYEAR_MINS_PER_TICK );
	//rx8900_fixed_timer_set_minutes(  1  );
						
	normalStepMode();			// Start stepping, never returns
	
	__builtin_unreachable();
		
}
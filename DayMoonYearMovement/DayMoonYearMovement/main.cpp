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

#define F_CPU 1000000
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

uint8_t buttonDown() {
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

// Set the RX8900 counts between INT events (MAX = 4195)
// A count can be a second or a minute (and others we don't care about) depending on TSEL bits.
// Count starts when TE bit enabled


void rx8900_fixed_timer_set_count( uint16_t c ) {	
	rx8900_reg_set( RX8900_TC0_REG , c & 0xff );		// Low byte of count value (in seconds)
	rx8900_reg_set( RX8900_TC1_REG , c / 0xff );		// High nibble of count value
}


void rx8900_fixed_timer_set_seconds( unsigned s ) {
	
	rx8900_fixed_timer_set_count(s);

	rx8900_reg_set( RX8900_EXTENTION_REG , 0b00111010); // Enable fixed timer, 1-second base clock

}

void rx8900_fixed_timer_set_minutes( unsigned s ) {
	
	rx8900_fixed_timer_set_count(s);
	
	rx8900_reg_set( RX8900_EXTENTION_REG , 0b00111011); // Enable fixed timer, 1-minute base clock

}



void rx8900_init() {
	
	i2c_init();
}

// Called anytime the RX8900 has seen low voltage (VLF)
// Sets all the control registers to known values
// Clears the flags, so make sure you save any that you need before calling
// Does not reset the time counter

void rx8900_setup(void) {

	// Enable /INT out on fixed timer, 2 second temp comp
	
	rx8900_reg_set( RX8900_CONTROL_REG , 0b01010000 );	
	
	// MOS switch closed, Voltage detector OFF. We have main battery wired to both Vcc and Bat so no backup detecting wanted. 
	
	rx8900_reg_set( RX8900_BACKUP_REG , 0b00001000 );
	
}


// We only need interrupts to wake us, so return from the ISR as quickly as possible. 

// Called when watchdog expires
EMPTY_INTERRUPT( WDT_vect);

// Called on button pin change interrupt
EMPTY_INTERRUPT( BUTTON_VECT );

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
	SETBIT( RX8900INT_PORT ,RX8900INT_BIT );	// Enable pull up
	while ( !GETBIT( RX8900INT_PORT ,RX8900INT_BIT  ) );
	sleep_cpu();	
	CLRBIT( RX8900INT_PORT ,RX8900INT_BIT );	// Disable pull up	
}



void setup() {
	
		
	// Enable interrupt on pin change.
	// " Any change on any enabled PCINT[5:0] pin will cause an interrupt. T"
	// Note that individual pins must also be enabled in the mask
	// Used to wake us every time the fixed-cycle timer pulls /INT low or when button is pressed (each must be enabled in PCMSK) 
	SETBIT( GIMSK  , PCIE );				//When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is enabled.
	
	// Set up to deep sleep between interrupts
	sleep_enable();							// How to do once to enable sleep
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// Lowest power, can wake on interrupts (pin change, WDT)
			

	// Activate the RTC pins
	rx8900_init();
	
	// Set RTC to send us A periodic interrupt on the /INT pin
	rx8900_setup();
	
//	rx8900_fixed_timer_set_seconds( 2.5 * 60 );		// 2 1/2 mins per quadrant - 10 minutes all the way around
	
}


// Spin at full speed the specified number of ticks (0-3600)
// It takes about 6 minutes to go all the way around (3600 steps * 100ms/step)
// (not exact because it depends on the watchdog timer to measure 48ms per half step)
// Returns the current phase when done (0 or 1)

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
// This version does not use any interrupts. 

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


// Run the clock normally, moving the hand one step per tick. Runs forever. 
// The tick is generated by the RX8900 ~INT pin, and the frequency that this
// pin generates a tick is controlled by the Timer Counter (TC) register and TSEL bits.
// Assumes interrupts on, global pin change enabled, /INT pin pulled up, RX8900 programmed to generate /INT signal, and /INT pin mask enabled. 
// Never returns.

void normalStepMode() {
	
	uint8_t phase =0;
	 
	while (1) {
		
		sleep_cpu();			// Wait for /INT to wake us
				
		CLRBIT( RX8900INT_PORT , RX8900INT_BIT );	// Disable pull up. /INT Will be pulled low by RTC for ~7ms, so no need to waste power though the pull up.

		// Disable /INT interrupt here to avoid spurious interrupt when /INT rises from waking us from WDT sleep.
		CLRBIT( PCMSK , RX8900INT_PCMSK );		// Disable interrupt on /INT pin change. This will prevent us from waking from the WDT delay sleep when this pin floats after the RTC stops pulling it low.
		SETBIT( DIDR0 , ADC1D );				// Turn off digital input buffer on the pin connected to /INT so will not waste power when it floats when RTC stops pulling it low.

		movement_mid_on(phase);
		phase = !phase;		
		
		sleep16ms();
		SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// OK, /INT should be reset by the time we get here, so turn on the pull up again. This will give it time to pull the voltage back up by the time we are ready to sleep again.
		
				
		sleep32ms();
		movement_mid_off();

		CLRBIT( DIDR0 , ADC1D );				// Enable digital input buffer on the pin connected to /INT so we will see when RTC pulls it low
		SETBIT( PCMSK , RX8900INT_PCMSK );		// Enable interrupt on /INT pin change to wake us when RTC says we are ready for next tick.
					
					
	}	
	
	__builtin_unreachable();
	
}

// Low power wait for the button to be pressed. Assumes no other interrupts running.
// Assumes interrupts enabled and PCIE enabled in GIMSK, but no other interrupts on.
// Returns with button interrupt disabled. 

void waitButtonPress() {
	
	// Enable the hardware

	sleep32ms();					   // Give pull-up time to raise voltage on pin to avoid spurious change	
	
		
	while (!buttonDown());			// Wait for button to be pressed before starting.
	
	disableButton();
	
	sei();
	
	
}

#define TICKS_PER_ROTATION (3600)

#define LUNARMONTH_SECS (2551443UL)			//https://www.justintools.com/unit-conversion/time.php?k1=lunar-months&k2=seconds
#define LUNARMONTH_SECS_PER_TICK ( LUNARMONTH_SECS / TICKS_PER_ROTATION )

#define SOLARDAY_SECS (60UL*60*24)			
#define SOLARDAY_SECS_PER_TICK ( SOLARDAY_SECS/ TICKS_PER_ROTATION )

#define TROPICALYEAR_SECS ()
#define TROPICALYEAR_SECS_PER_TICK ( SOLARDAY_SECS/ TICKS_PER_ROTATION )



// TODO: This does not work out exactly, so do a Bresenham's style error track and adjust 

int main(void)
{
	
	
	// *** First set up all pins and pull-ups
	
	SETBIT( BUTTON_PORT , BUTTON_BIT);				// Enable button pull-up. Button connects pin to ground.
	
	SETBIT( RX8900INT_PORT , RX8900INT_BIT );		// Enable pull-up on /INT pin. This is open-drain output on the RX8900 so we need to pull it up with the ATTINY.
	
	SETBIT( UNUSEDPIN_DDR , UNUSEDPIN_BIT );		// Drive the unused pin low. This pin is connected so we don't want it to float and waste power. 
	
	i2c_init();										// This activates the pull-ups on the i2c SDA and SCL pins. 
	
	_delay_ms(10);		// Give the pull-ups a chance to pull the voltages. Don't use the WDT-based waits because they might get interrupted by the above changing pins. 
	
	// *** Global interrupt preparation
	
	// Enable interrupt on pin change.
	// " Any change on any enabled PCINT[5:0] pin will cause an interrupt. T"
	// Note that individual pins must also be enabled in the mask
	// Used to wake us every time the fixed-cycle timer pulls /INT low or when button is pressed (each must be enabled in PCMSK)
	SETBIT( GIMSK  , PCIE );				//When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is enabled.
	
	sei();			// And finally ready to actually enable interrupts. Still need to enable individual pins in the mask as needed.
	
	// *** Enable sleeping mode
	
	sleep_enable();							// Do once to enable sleep forever
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// Lowest power, can wake on interrupts (pin change, WDT)		
			

	// *** Wait for a button press to get us started		
	
	SETBIT( PCMSK , BUTTON_PCMSK );	   // Enable change interrupt on this pin. You must also sei() and enable PCIE in GIMSK to enable all pin change interrupts
	
	sleep_cpu();						// Sleep - we will wake up when button state changes.

	CLRBIT( PCMSK , BUTTON_PCMSK );	   // Disable the interrupt that got us here.

	
	CLRBIT( BUTTON_PORT , BUTTON_BIT );		// Disable the pull-up
	SETBIT( BUTTON_DDR , BUTTON_BIT );		// Drive button pin low so it will not use any power (we never need to check it again)
											// Note we do not leave pull-up on since then would use power as long as button is pressed. 

	// *** Show we woke	
		
	spin(TICKS_PER_ROTATION/2);			// One half time around to make sure we are working
	
	
	// *** Enable interrupt on /INT signal from RTC

	SETBIT( PCMSK , RX8900INT_PCMSK );	   // Enable change interrupt on /INT pin change from RTC. 

	
	// *** Program RTC to generate /INT signal period
	
	//rx8900_fixed_timer_set_seconds( SOLARDAY_SECS_PER_TICK );		
	//rx8900_fixed_timer_set_seconds( LUNARMONTH_SECS_PER_TICK );
	rx8900_fixed_timer_set_seconds( TROPICALYEAR_SECS_PER_TICK );
			
			
	normalStepMode();			// Start stepping, never returns
	
	__builtin_unreachable();
		
}


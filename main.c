/**
 * main.c - EGB240 Digital Voice Recorder Skeleton Code
 *
 * This code provides a skeleton implementation of a digital voice
 * recorder using the Teensy microcontroller and QUT TensyBOBv2
 * development boards. This skeleton code demonstrates usage of
 * the EGB240DVR library, which provides functions for recording
 * audio samples from the ADC, storing samples temporarily in a
 * circular buffer, and reading/writing samples to/from flash
 * memory on an SD card (using the FAT file system and WAVE file
 * format.
 *
 * This skeleton code provides a recording implementation which
 * samples CH0 of the ADC at 8-bit, 15.625kHz. Samples are stored
 * in flash memory on an SD card in the WAVE file format. The
 * filename is set to "EGB240.WAV". The SD card must be formatted
 * with the FAT file system. Recorded WAVE files are playable on
 * a computer.
 *
 * LED4 on the TeensyBOBv2 is configured to flash as an
 * indicator that the programme is running; a 1 Hz, 50 % duty
 * cycle flash should be observed under normal operation.
 *
 * A serial USB interface is provided as a secondary control and
 * debugging interface. Errors will be printed to this interface.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */

 /************************************************************************/
 /* INCLUDED LIBRARIES/HEADER FILES                                      */
 /************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>

#include "serial.h"
#include "timer.h"
#include "wave.h"
#include "buffer.h"
#include "adc.h"

#define TOP 255 // Top value of timer as 8 bit

/************************************************************************/
/* ENUM DEFINITIONS                                                     */
/************************************************************************/
enum {
	DVR_STOPPED,
	DVR_RECORDING,
	DVR_PLAYING
};

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
volatile uint16_t pageCount = 0;	// Page counter - used to terminate recording
volatile uint16_t newPage = 0;	// Flag that indicates a new page is available for read/write
volatile uint8_t stop = 0;		// Flag that indicates playback/recording is complete

volatile uint32_t s = 0;        // Number of samples in open wave file
volatile uint16_t s_final = 0;  // Final number of samples at the end < 512
volatile uint8_t release = 1;   // Button release flag

volatile uint8_t pb0, pb1, pb2; // Button stability steps
volatile uint8_t stable = 0;    // Button stable flag

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
void pageFull();
void pageEmpty();

/************************************************************************/
/* INITIALISATION FUNCTIONS                                             */
/************************************************************************/

// Initialise PLL (required by USB serial interface, PWM)
void pll_init() {
	PLLFRQ = 0x6A; // PLL = 96 MHz, USB = 48 MHz, TIM4 = 64 MHz
}

// Configure system clock for 16 MHz
void clock_init() {
	CLKPR = 0x80;	// Prescaler change enable
	CLKPR = 0x00;	// Prescaler /1, 16 MHz
}

// Initialise PORTD for pushbuttons and PORTF for corresponding LEDs
void button_init(){
	DDRF = DDRF & 0b00001111;   // Set bits 7-4 pushbuttons as inputs
	DDRD |= 0b01110000;         // Set bits 6-4 LEDs as outputs
	PORTD &= 0b00001111;        // Turn all LEDs off
}

// Initialise PWM
void pwm_init(){
	DDRB |= 0b01000000; // Set PORTB 6 as an output (JOUT)
	TIMSK4 |= (1 << OCIE4A); // enable interrupt for CompareA

	TCCR4A |= (1 << COM4A1)|(1 << COM4A0)|(1 << PWM4A); // Enable SET on compare; enable PWM
	OCR4A = TOP;    // set top to 0xFF (255)

	TCNT4 = 0x00;  // reset counter
}

// Initialise debouncing timer
void db_init(){
    TIMSK3 |= (1 << TOIE3); // Enable interrupt on overflow

    TCCR3B = 0x01; // Prescaler /1 no prescaling
}

// Initialise DVR subsystems and enable interrupts
void init() {
	cli();			// Disable interrupts
	clock_init();	// Configure clocks
	button_init();  // Initialise buttons
	pwm_init();     // Initialise PWM
	db_init();      // Initialise debouncing timer
	pll_init();     // Configure PLL (used by Timer4 and USB serial)
	serial_init();	// Initialise USB serial interface (debug)
	timer_init();	// Initialise timer (used by FatFs library)
	buffer_init(pageFull, pageEmpty);  // Initialise circular buffer (must specify callback functions)
	adc_init();		// Initialise ADC
	sei();			// Enable interrupts

	// Must be called after interrupts are enabled
	wave_init();	// Initialise WAVE file interface
}

/************************************************************************/
/* CALLBACK FUNCTIONS FOR CIRCULAR BUFFER                               */
/************************************************************************/

// CALLED FROM BUFFER MODULE WHEN A PAGE IS FILLED WITH RECORDED SAMPLES
void pageFull() {
	if(!(--pageCount)) {
		// If all pages have been read
		adc_stop();		// Stop recording (disable new ADC conversions)
		stop = 1;		// Flag recording complete
	} else {
		newPage = 1;	// Flag new page is ready to write to SD card
	}
}

// CALLED FROM BUFFER MODULE WHEN A NEW PAGE HAS BEEN EMPTIED
void pageEmpty() {
	// TODO: Implement code to handle "page empty" callback
	if (s >= 512){  // If the reader has reached the last page of samples
		s = s - 512;    // Stepping through written samples
	} else {
	    s_final = s;    // set final number of samples to be read
		s = 0;
	}
	if (!s){
		stop = 1;   // Once all samples are read, flag to stop
	} else {
		newPage = 1;    // Otherwise signify there a new page to be written
	}
}

/************************************************************************/
/* RECORD/PLAYBACK ROUTINES                                             */
/************************************************************************/

// Initiates a record cycle
void dvr_record() {
	buffer_reset();		// Reset buffer state

	pageCount = 305;	// Maximum record time of 10 sec
	newPage = 0;		// Clear new page flag

	wave_create();		// Create new wave file on the SD card
	adc_start();		// Begin sampling

	// TODO: Add code to handle LEDs
}

// TODO: Implement code to initiate playback and to stop recording/playback.
void dvr_playback() {
	buffer_reset(); // Reset buffer state

	newPage = 0;    // Clear new page flag

	s = wave_open();    // Open wave file and store number of samples
	wave_read(buffer_writePage(), 512);
	wave_read(buffer_writePage(), 512); // Initially fill two pages of buffer to catch up with player
	s = s - 1024;   // Reduce sample count by initial amount read

	TCCR4B |= 0b00000110;   // Prescale timer to /32 for correct playback speed
	printf("INITIALISE PLAYBACK\n");
}

/************************************************************************/
/* MAIN LOOP (CODE ENTRY)                                               */
/************************************************************************/
int main(void) {
	uint8_t state = DVR_STOPPED;	// Start DVR in stopped state

	// Debouncing variable init
	uint8_t pb = 0;
	// Button value, previous value and edge value below
    uint8_t REC_val = 0, pREC = 0, RECe = 0;
    uint8_t PLY_val = 0, pPLY = 0, PLYe = 0;

	// Initialisation
	init();

	// Loop forever (state machine)
    for(;;) {

        // Debouncing loop
        if (stable){ // Check if stable to set ~PINF
            pb = pb0;
        }

        // Set record and play button
        REC_val = pb & 0b00100000;
        PLY_val = pb & 0b00010000;

        // Find transition of button low to high
        RECe = ((pREC == 0) && (REC_val != 0));
        PLYe = ((pPLY == 0) && (PLY_val != 0));

		// Switch depending on state
		switch (state) {
			case DVR_STOPPED:
				PORTD |= 0b01000000;    // LED3 (Blue) signifying stopped
				PORTD &= ~(0b00110000); // Turn off all LEDs

				// TODO: Implement button/LED handling for record/playback/stop

				// TODO: Implement code to initiate recording
				// if ( /* RECORD REQUESTED */ ) {
				//	printf("Recording...");	// Output status to console
				//	dvr_record();			// Initiate recording
				//	state = DVR_RECORDING;	// Transition to "recording" state
				// }

                // if buttons are still held, they will not count as released
				if (PINF & 0b00100000 && PINF & 0b00010000){
                    release = 1;
				}

				// Release variable is in flow control to ensure functionality only works if the button has been released
				// Check for 'edge' of button press after checking stability for debouncing
				if (RECe && release){
					dvr_record();           // Execute initial recording preparations
					state = DVR_RECORDING;  // Switch state
				} else if (PLYe && release){
					dvr_playback();         // Execute initial playback preparations
					state = DVR_PLAYING;    // Switch state
				}

				break;
			case DVR_RECORDING:
				PORTD |= 0b00100000;    // LED2 (Red) signifying record
				PORTD &= ~(0b01010000); // Turn off all LEDs
				release = 0;    // switch release flag for checking

				// TODO: Implement stop functionality
				// if ( /* STOP REQUESTED */ ) {
				//	pageCount = 1;	// Finish recording last page
				// }

                // if the stop button is pressed, end recording
				if (~PINF & 0b01000000){
                    release = 1;    // count as button released
					pageCount = 1;  // last page
				}

				// Write samples to SD card when buffer page is full
				if (newPage) {
					newPage = 0;	                    // Acknowledge new page flag
					wave_write(buffer_readPage(), 512);
				} else if (stop) {
					// Stop is flagged when the last page has been recorded
					stop = 0;							// Acknowledge stop flag
					wave_write(buffer_readPage(), 512);	// Write final page
					wave_close();						// Finalise WAVE file
					printf("DONE!\n");					// Print status to console
					state = DVR_STOPPED;				// Transition to stopped state
				}
				break;
			case DVR_PLAYING:
				PORTD |= 0b00010000;    // LED1 (Green) signifying playback
				PORTD &= ~(0b01100000); // Turn off all LEDs
				release = 0;    // switch release flag for checking

                // if the stop button is pressed, end playback
				if (~PINF & 0b01000000){
                    release = 1;    // count as released
					s = 0;  // last sample
				}

				// TODO: Implement playback functionality
				if (newPage){
					newPage = 0;                        // Acknowledge new page flag
					wave_read(buffer_writePage(), 512); // Write wave file into one buffer page
					printf("PLAYING...\n");
				} else if (stop){
					stop = 0;                           // Acknowledge stop flag
					wave_read(buffer_writePage(), s_final); // Write remaining samples into page < 512
					wave_close();                       // Close wave file
					TCCR4B = 0x00;                      // Reset timer 4
					TCNT4 = 0x00;                       // Reset counter for timer 4
					printf("DONE!\n");
					state = DVR_STOPPED;                // Transition into stopped state
				}
				break;
			default:
				// Invalid state, return to valid idle state (stopped)
				printf("ERROR: State machine in main entered invalid state!\n");
				state = DVR_STOPPED;
				break;

		} // END switch(state)
		// Set previous value of button to current value post loop for finding low-high transition
		pREC = REC_val;
		pPLY = PLY_val;

	} // END for(;;)

}

// Interrupt when timer overflows
ISR(TIMER3_OVF_vect){
    // Read value of buttons multiple times
    pb2 = pb1;
    pb1 = pb0;
    pb0 = ~PINF;

	// If last 3 PB readings the same, consider this stable
	if ((pb0 == pb1) && (pb0 == pb2)){
		stable = 1;
	} else {
	    stable = 0;
	}
}

// Interrupt when compare value is set
ISR(TIMER4_COMPA_vect){
	TCNT4 = buffer_dequeue();   // Remove sample from buffer and into compare register
	PORTB ^= (1 << PINB6);      // Output pwm from compared value
}


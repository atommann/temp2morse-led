/**************************************************************************
* Morse code thermometer
* (c) 2012 atommann
*
* Todo:
*     1. Display negative temperature
*     2. Use internal 128KHz clock source
* 
**************************************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 1000000
#include <util/delay.h>

#include "onewire.h"
#include "ds18x20.h"

#define MAXSENSORS 1

#define OW_PWR_PIN  PB1
#define OW_PWR_IN   PINB
#define OW_PWR_OUT  PORTB
#define OW_PWR_DDR  DDRB

#define LED_PIN  PB3
#define LED_IN   PINB
#define LED_OUT  PORTB
#define LED_DDR  DDRB

// symbolic definitions of numbers to control length of dashes and dots
#define DASH 600
#define DOT  200

// An initialised array of bytes containing the dot/dash codes for each figure/letter
// 0: dit
// 1: dash
// The last '1' is used as end-mark(delimiter)
const uint8_t codes[10] PROGMEM = {
0xFC, // 0, 1111 1100 -----
0x7C, // 1, 0111 1100 .----
0x3C, // 2, 0011 1100 ..---
0x1C, // 3, 0001 1100 ...--
0x0C, // 4, 0000 1100 ....-
0x04, // 5, 0000 0100 .....
0x84, // 6, 1000 0100 -....
0xC4, // 7, 1100 0100 --...
0xE4, // 8, 1110 0100 ---..
0xF4, // 9, 1111 0100 ----.
};


// globals
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t temperature_string[] = "--.-";
volatile uint8_t display_flag; 
uint8_t nSensors;

// function declaration
void init(void);
void sleep_init(void);
double getTempF(double v10bit, double pdRes);
void double2string (double actualTemp, uint8_t* string);
void deci2string (uint16_t decicelsius, uint8_t* string);
void morse_emit(uint8_t index);


// INT0 ISR
ISR(INT0_vect)
{ 
    display_flag = 1;
}

void morse_emit(uint8_t index)
{
    uint8_t shift;

    shift = pgm_read_byte(codes + index);

    do {
        LED_OUT |= _BV(LED_PIN); // LED on
	
        if(shift & 0x80)
            _delay_ms(DASH);
        else
            _delay_ms(DOT);
	
        LED_OUT &= ~_BV(LED_PIN); // LED off
	
        _delay_ms(DOT);
        shift <<= 1;
    } while(shift != 0x80);

    _delay_ms(DOT);
}


static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	// uart_puts_P( NEWLINESTR "Scanning Bus for DS18X20" NEWLINESTR );
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			// uart_puts_P( "No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			// uart_puts_P( "Bus Error" NEWLINESTR );
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	
	return nSensors;
}


/***************************************************************************
* double2string
* convert a double to a string and place it in a pre-allocated space
***************************************************************************/
void double2string (double actualTemp, uint8_t* string)
{
	int temp;

	cli(); // atomic operation

	/* prep the string */
	string[2] = '.';
	string[4] = '\0';

	temp=(int16_t)(actualTemp * 10.0);   //to include decimal point for display
	if((actualTemp*10.0 - temp) >= 0.5) temp=temp+1;

	if(temp < 0)
	{
	  temp *= -1;
	}
	
	string[3] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[1] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[0] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	sei();
}

void deci2string (uint16_t decicelsius, uint8_t* string)
{
	int temp;

	cli(); // atomic operation

	/* prep the string */
	string[2] = '.';
	string[4] = '\0';

	temp=decicelsius;   //to include decimal point for display

	if(temp < 0)
	{
	  temp *= -1;
	}
	
	string[3] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[1] = ((uint8_t)(temp%10)) | 0x30;
	temp=temp/10;
	
	string[0] = ((uint8_t)(temp%10)) | 0x30;
	if (string[0] == 0x30) // suppress the leading zeros
		string[0] = ' ';
	temp=temp/10;
	
	sei();
}

void init(void) 
{
    DDRB |= _BV(LED_PIN) | _BV(OW_PWR_PIN);
    PORTB |= _BV(OW_PWR_PIN) | _BV(PB2) | _BV(PB4);

    GIMSK |= _BV(INT0);                  // enable INT0
    MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // INT0 on low level
}

void sleep_init(void)
{
    ACSR |= _BV(ACD);                    // disable the analog comparator
    ADCSRA &= ~_BV(ADEN);                // disable ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    //turn off the brown-out detector.
    //must have an ATtiny45 or ATtiny85 rev C or later for software to be able to disable the BOD.
    //current while sleeping will be <0.5uA if BOD is disabled, <25uA if not.
    cli();
    /*
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    */
    sei();                         //ensure interrupts enabled so we can wake up again
    sleep_cpu();                   //go to sleep
    cli();                         //wake up here, disable interrupts
    sleep_disable();               
    sei();                         //enable interrupts again (but INT0 is disabled from above)
}

void convert_show(void)
{
    uint8_t i;
    int16_t decicelsius;
    uint8_t error;
    
    PORTB |= _BV(OW_PWR_PIN); // turn on power supply for DS18B20
    
    for ( i = nSensors; i > 0; i-- ) {
        if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, &gSensorIDs[i-1][0] ) == DS18X20_OK ) {
	    _delay_ms( DS18B20_TCONV_12BIT );
	    //uart_puts_P( "Sensor# " );
	    
	    if ( DS18X20_read_decicelsius( &gSensorIDs[i-1][0], &decicelsius) == DS18X20_OK ) {
		//uart_put_temp( decicelsius );
		deci2string(decicelsius, temperature_string);
	    } else {
		//uart_puts_P( "CRC Error (lost connection?)" );
		error++;
	    }
	    //uart_puts_P( NEWLINESTR );
	}
	else {
	    //uart_puts_P( "Start meas. failed (short circuit?)" );
	    error++;
        }
    }
    
    morse_emit(temperature_string[0] - 0x30);
    morse_emit(temperature_string[1] - 0x30);
    
    PORTB &= ~ _BV(OW_PWR_PIN); // turn off power supply for DS18B20
}

int main(void)
{
    init();
    nSensors = search_sensors();
    sleep_init();
    
    while (1) {
        if (display_flag) {
            convert_show();
            display_flag = 0;
        }
	
        sleep_cpu();
        sleep_disable();
    }
}

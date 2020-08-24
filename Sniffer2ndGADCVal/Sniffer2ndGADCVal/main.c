/*
 * Sniffer2ndGADCVal.c
 *
 * Created: 19-Aug-18 1:58:23 PM
 * Author : USER
 */ 

#define F_CPU 16000000UL  // 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>



#define BAUD 9600                           // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)    // set baudrate value for UBR

volatile uint8_t adcValueBufferValidFlag = 0;
volatile uint16_t adcValueBuffer = 0;
volatile uint32_t accumulator = 0;
volatile uint16_t sample = 0;
volatile uint16_t average = 0;

void adc_init()
{
	
	ADMUX = 0x00;
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // pre scaling by 128
	ADMUX |= (1<<REFS0);                     // AVCC as reference votage 
	ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution
	ADCSRA |= 1<<ADIE;          // enable interrupt 
	MCUCR |= 1<<SM0;             //Sleep mode     //Noise canceling mode
	MCUCR |= 1<<SE;              //sleep enable
	sei();
	ADCSRA |= 1<<ADSC;
}

// function to initialize UART
void uart_init (void)
{
	UBRRH=(BAUDRATE>>8);
	UBRRL=BAUDRATE;                         //set baud rate
	UCSRB|=(1<<TXEN);             //enable receiver and transmitter
	UCSRC|=(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);// 8bit data format
}

// function to send data - NOT REQUIRED FOR THIS PROGRAM IMPLEMENTATION
void uart_transmit (unsigned char data)
{
	while (!( UCSRA & (1<<UDRE)));            // wait while register is free
	UDR = data;                             // load data in the register
}

void SendString(char mydata[20])
{
	int i;
	for(i=0;i<strlen(mydata);i++)
	{
		uart_transmit(mydata[i]);
	}
}

void timer0_init()
{
	TCNT0=0x00;
	TCCR0 = (1<<CS00) | (1<<CS02); // presceler of 1024
}

int main(void)
{
	uart_init();
	adc_init();
	timer0_init();
	uint8_t timerOverflowCount=0;
	uint16_t adcValueCache; // Local variable which will hold the ADC value until it is completely transmitted.
	char mychar[5];
	while (1)
	{
		// Wait for the ISR to signal that a new value is available:
		while ( adcValueBufferValidFlag == 0 ) {
		}
		
		while ((TIFR & 0x01) == 0);
		TCNT0 = 0x00;
		TIFR=0x01; //clear timer1 overflow flag
		timerOverflowCount++;
		
		if (timerOverflowCount>=62)
		{
			adcValueBufferValidFlag = 0; // Re-set flag. Will be set again by the ISR when a new ADC value becomes available.

			// Make sure that we read the buffered value atomically:
			cli();
	
			adcValueCache = average;

			sei();
			sprintf(mychar,"%04d",adcValueCache);  //This will convert integer into ASCII array
			SendString(" Analog Val:");
			SendString(mychar);            //Send it
			//uart_transmit(13);
		
			SendString("\n\r"); //Line break
			//_delay_ms(1000);
			timerOverflowCount=0;
		}
		
	}
}
ISR(ADC_vect)
{
	uint8_t theLowADC = ADCL;
	uint16_t theTenBitResults = ADCH<<8 | theLowADC;
	//adcValueBuffer = ADC;
	
	accumulator += theTenBitResults;
	sample++;
	if(sample == 100)
	{
		average = accumulator/100;
		accumulator = 0;
		sample = 0;
		adcValueBufferValidFlag = 1; // This signals that the ADC provided a new value for the code outside the ISR.
	}
	ADCSRA |= 1<<ADSC;
}


/*
 * Sniffer2ndGR0Calculation.c
 * Created: 28-Jul-18 1:12:15 PM
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
volatile uint8_t R0calFlag = 0;

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

void timer1_init()
{
	OCR1A = 7812;

	TCCR1B |= (1 << WGM12);
	// Mode 4, CTC on OCR1A

	TIMSK |= (1 << OCIE1A);
	//Set interrupt on compare match

	TCCR1B |= (1 << CS12) | (1 << CS10);
	// set prescaler to 1024 and start the timer


	sei();
	// enable interrupts
}

ISR (TIMER1_COMPA_vect)
{
	// action to be done every 10ms
	R0calFlag = 1;
}


int main(void)
{
	uart_init();
	adc_init();
	timer0_init();
	timer1_init();
	float sensor_volt; //Define variable for sensor voltage
	float RS_air; //Define variable for sensor resistance
	float R0; //Define variable for R0
	float sensorValue=0; //Define variable for analog readings
	uint8_t R0calCount = 0;
	char mychar[5];

	
	while (1)
	{
		if(R0calFlag == 1)
		{
			sensorValue = sensorValue + average; //Add analog values of sensor 50 times
			uart_transmit('.');
			R0calFlag = 0;
			R0calCount++;
		}
		if (R0calCount == 50)
		{
			sensorValue = sensorValue / 50; //Take average of readings
			sensor_volt = sensorValue * (5.0 / 1023.0); //Convert average to voltage
			RS_air = ((5.0 * 9.76) / sensor_volt) - 9.76; //Calculate RS in fresh air  //RS = [(VC x RL) / VRL] - RL
			R0 = RS_air / 4.4; //Calculate R0, for fresh air:  RS / R0 = 4.4 ppm(mq4) , 9.83(mq2)
			sprintf(mychar,"%.2f",R0);  //This will convert integer into ASCII array
			SendString("\n\r"); //Line break
			SendString("R0=");
			SendString(mychar);
			SendString("\n\r"); //Line break
			R0calCount = 0;
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
		//adcValueBufferValidFlag = 1; // This signals that the ADC provided a new value for the code outside the ISR.
	}
	ADCSRA |= 1<<ADSC;
}




/*
 * Sniffer2ndGFullFunction.c
 *
 * Created: 12-Sep-18 4:45:58 PM
 * Author : USER
 */ 

#define F_CPU 16000000UL  // 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>


#define Buzzer_PIN PD7
#define WhiteLED_PIN PB4
#define REDLED_PIN PB3
#define GreenLED_PIN PB1 
#define BlueLED_PIN PB5
#define AdapterInput_PIN PD6
#define GASTHRES 100

volatile uint32_t accumulator = 0;
volatile uint16_t sample = 0;
volatile uint16_t average = 0;
volatile uint8_t PPMcalFlag = 0;
volatile uint8_t buzzerFlag = 0;
volatile uint8_t timeFlag = 0;

uint16_t timeCounter = 0;
float R0 = 5.40;
float m = -0.382, b = 1.168; //for methane 

void adc_init()
{
	
	ADMUX = 0x00 ;          //adc channel 0
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // pre scaling by 128
	ADMUX |= (1<<REFS0);                     // AVCC as reference votage
	ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution
	ADCSRA |= 1<<ADIE;          // enable interrupt
	MCUCR |= 1<<SM0;             //Sleep mode     //Noise canceling mode
	MCUCR |= 1<<SE;              //sleep enable
	sei();
	ADCSRA |= 1<<ADSC;
}


void timer1_init()
{
	OCR1A = 15624;     //1 second compare
	TCCR1B |= (1 << WGM12);    // Mode 4, CTC on OCR1A
	TIMSK |= (1 << OCIE1A);    //Set interrupt on compare match
	TCCR1B |= (1 << CS12) | (1 << CS10);  // set prescaler to 1024 and start the timer
	sei();             // enable interrupts
	
}

ISR (TIMER1_COMPA_vect)
{
	// action to be done every 10ms
	PPMcalFlag = 1;
	buzzerFlag = 1;
	timeFlag = 1;
}


int main(void)
{
	adc_init();
	timer1_init();
	
	DDRD |= (1 << Buzzer_PIN);
	DDRB |= (1 << WhiteLED_PIN);
	DDRB |= (1 << GreenLED_PIN);
	DDRB |= (1 << REDLED_PIN);
	DDRB |= (1 << BlueLED_PIN);
	DDRD &= ~(1<<AdapterInput_PIN);//Makes firs pin of PORTD as Input
	PORTB = 0x00;
	PORTD = 0x00;
	
	while(timeCounter <= 60)
	{
		if (timeFlag == 1)
		{
			timeCounter++;
			PORTB ^= 1 <<GreenLED_PIN;
			timeFlag = 0;
		}
	}
	PORTD ^= 1 << Buzzer_PIN;
	_delay_ms(300);
	float sensor_volt; //Define variable for sensor voltage
	float RS_gas; //Define variable for sensor resistance
	float ratio; //Define variable for ratio
	float sensorValue = 0; //Define variable for analog readings
	double ppm_log = 0;
	double ppm = 0;
	uint8_t buzzDuration = 0;
	
	while (1)
	{
		if(PPMcalFlag == 1)
		{
			sensorValue = average; //Read analog values of sensor
			sensor_volt = sensorValue * (5.0 / 1023.0); //Convert analog values to voltage
			RS_gas = ((5.0 * 9.76) / sensor_volt) - 9.76; //Get value of RS in a gas //RS = [(VC x RL) / VRL] - RL
			ratio = RS_gas / R0;   // Get ratio RS_gas/RS_air

			ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
			ppm = pow(10, ppm_log); //Convert ppm value to log scale
			
			if (ppm > 9999)
			{
				ppm = 10000;
			}
			
			PPMcalFlag = 0;
		}
		
		if (ppm > GASTHRES)
			{
				PORTB &= ~(1 <<WhiteLED_PIN);
				PORTB &= ~(1<<BlueLED_PIN); //Turns OFF LED
				while (buzzDuration <= 8)
				{
					if (buzzerFlag == 1)
					{
						PORTB ^= 1 << REDLED_PIN;
						PORTD ^= 1 << Buzzer_PIN;
						buzzerFlag = 0;
						buzzDuration++;
					}

				}
				if (buzzDuration > 8)
				buzzDuration = 0;
			}
			
			
		else
			{
				if(PIND & (1<<AdapterInput_PIN) ) //If switch is pressed				
					{
						PORTB &= ~(1<<BlueLED_PIN); //Turns OFF LED
						PORTB |= (1 << WhiteLED_PIN);						
					}
				else
					{	
						PORTB &= ~(1 <<WhiteLED_PIN);
						PORTB |= (1<<BlueLED_PIN); //Turns ON LED
					}
				PORTD &= ~(1 << Buzzer_PIN);
				PORTB &= ~(1 << REDLED_PIN);
				PORTB &= ~(1 << GreenLED_PIN);
			}	


	}
}
ISR(ADC_vect)
{
	uint8_t theLowADC = ADCL;
	uint16_t theTenBitResults = ADCH<<8 | theLowADC;
	
	accumulator += theTenBitResults;
	sample++;
	if(sample == 100)
	{
		average = accumulator/100;
		accumulator = 0;
		sample = 0;
	}
	ADCSRA |= 1<<ADSC;
}


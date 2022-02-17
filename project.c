/*
 * project.c
 *
 *  Created on: Sep 15, 2021
 *      Author: as
 */
#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>


unsigned char g_second=0;
unsigned char g_minutes=0;
unsigned char minutes=0;
unsigned char g_hour=0;
unsigned char hour=0;
unsigned char second=0;
unsigned char flag=0;



ISR(TIMER1_COMPA_vect){
     flag=1;
   }
ISR(INT0_vect){
	 g_second=0;
     g_minutes=0;
	 minutes=0;
	 g_hour=0;
	 hour=0;
	 second=0;
	 flag=0;
}
ISR(INT1_vect){
	TCCR1B&=~(1<<CS10);
	TCCR1B&=~(1<<CS11);
	TCCR1B&=~(1<<CS12);
}
ISR(INT2_vect){
	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
}
void Timer1_Init(void){
	TCNT1 = 0;
	OCR1A=1000;
	TIMSK|=(1<<OCIE1A);
	TCCR1A=(1<<FOC1A);
	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
}
void INT0_Init(void){
	SREG  &= ~(1<<7);
	DDRD  &= (~(1<<PD2));
	PORTD|=(1<<PD2);
	GICR|=(1<<INT0);
	MCUCR=(1<<ISC01);
	SREG  |= (1<<7);
}
void INT1_Init(void){
	SREG  &= ~(1<<7);
	DDRD  &= (~(1<<PD3));
	GICR|=(1<<INT1);
	MCUCR|=(1<<ISC11)|(1<<ISC10);
	SREG  |= (1<<7);
}
void INT2_Init(void){
	SREG  &= ~(1<<7);
	DDRB  &= (~(1<<PB2));
	PORTB|=(1<<PB2);
	GICR|=(1<<INT2);
    MCUCSR|=(1<<ISC2);
	SREG  |= (1<<7);
}

int main (void){
DDRC|=0x0f;
PORTC&=~0x0f;
DDRA|=0x3f;
PORTA=0x3f;
SREG|=(1<<7);
INT0_Init();
INT1_Init();
INT2_Init();
Timer1_Init();

while(1){
	PORTC = (PORTC & 0xF0) | (g_second & 0x0F);
	PORTA=0x01;
    _delay_us(500);
    PORTA=0x00;
    PORTC = (PORTC & 0xF0) | (second & 0x0F);
	PORTA=0x02;
	_delay_us(500);
    PORTA=0x00;
    PORTC = (PORTC & 0xF0) | (g_minutes & 0x0F);
    PORTA=0x04;
    _delay_us(500);
    PORTA=0x00;
    PORTC = (PORTC & 0xF0) | (minutes & 0x0F);
    PORTA=0x08;
    _delay_us(500);
    PORTA=0x00;
    PORTC = (PORTC & 0xF0) | (g_hour & 0x0F);
    PORTA=0x10;
    _delay_us(500);
    PORTA=0x00;
    PORTC = (PORTC & 0xF0) | (hour & 0x0F);
    PORTA=0x20;
    _delay_us(500);
    PORTA=0x00;

	 if(flag==1){
		 if(g_minutes==9&&minutes==5){
			 if(g_hour==9){
							 hour++;
							 g_hour=0;
							 if(hour==6)
								 hour=0;
						 }
						 else
							 g_hour++;
				//		 g_tick2=0;
		 }
		 if(g_second==9&&second==5){
			// g_tick2++;
			 if(g_minutes==9){
				 minutes++;
				 g_minutes=0;
				 if(minutes==6)
					 minutes=0;
			 }
			 else
				 g_minutes++;
			 //g_tick=0;
		 }
					 if(g_second==9){
						 	second++;
						     //g_tick++;
						 	g_second=0;
						 	if(second==6){
						 		second=0;
						 	}
					 }
				     else{
					       // g_tick++;
				    	    g_second++;

					     }
flag=0;

}
}
}



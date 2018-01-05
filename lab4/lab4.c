/*ECE 473, lab wrriten by Mohammed Alarifi*/

#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>
#include "hd44780.h"
#include"music.h"

uint8_t counter = 0;
uint16_t  SavedPORTA_Value = 0;
uint16_t NotTouched=0xFF;
uint8_t  nothing = 1;
uint16_t  spdr_to_encoder;
uint8_t  BARAGRAPH = 0;
uint8_t  OE1 = 0;//old value of encoder 1
uint8_t  OE2 = 0; //old value of encoder 2
uint8_t  encoder1 = 0;
uint8_t  encoder2 = 0;
uint8_t   sec=0;
uint8_t   minutes=0;
uint8_t   hours=12;
uint8_t 	i;
uint8_t		edit_hour;
uint8_t 	edit_minutes;
uint8_t		alarm_hours=12;
uint8_t		alarm_minutes=20;
uint8_t		alarm; //for displaying the alaram
uint8_t		alarmON=0; 
uint16_t tmp; //tmp variable to modify number for display
uint8_t cur_value; //current digit value to display
uint8_t cur_digit = 0; //current digit to display on
uint8_t lcd_fix=0;
char lcd_old[16];
char lcd_now[16];
uint8_t adcr=110;
uint16_t vol_n=100;
uint16_t vol_o=0;
uint8_t fix=0;

/******************************from lab 2 *********************************************/
uint8_t sev_seg_digits[10] = {
	0b11000000, //0
	0b11111001, //1
	0b10100100, //2
	0b10110000, //3
	0b10011001, //4
	0b10010010, //5
	0b10000011, //6
	0b11111000, //7
	0b10000000, //8
	0b10011000 //9
	
};

uint8_t sev_seg_digits_hex[16] = {
	0b11000000, //0
	0b11111001, //1
	0b10100100, //2
	0b10110000, //3
	0b10011001, //4
	0b10010010, //5
	0b10000011, //6
	0b11111000, //7
	0b10000000, //8
	0b10011000,  //9
	0x88,
	0x83,
	0xc6,
	0xA1,
	0x86,
	0x8E
};

/********************/
uint8_t decoder_select[6] = {
	0b00000000, //zero place
	0b00010000, //tens place
	0b00110000, //hundreds place
	0b01000000, //thousands place
	0b00100000, //center colon
	0b01110000  //hi-Z mode
};
/****************/
void display_alarm(){
cur_digit=0;

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups

tmp=alarm_minutes;
for(i=0; i<2 ; i++){
		cur_value = tmp % 10; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} 
/*PORTB=0b00100000;
PORTA=0x00;
_delay_ms(5);
*/

tmp=alarm_hours;
do {
		cur_value = tmp % 10; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} while (tmp >= 1);




	PORTB = 0x60; //switch encoder output to unused bit to remove ghosting





}

/*******************************/
void display_digits() 
{
	cur_digit=0;

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups


	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
	/*do {
		cur_value = tmp % 10; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} while (tmp >= 1);
*/

/*tmp=minutes;


cur_value = tmp % 10; 
PORTB=0x00;
PORTB|=decoder_select[0];
PORTA = 0xFF; //clear PORTA
PORTA = sev_seg_digits[cur_value]; //display digit
*/
tmp=minutes;
for(i=0; i<2 ; i++){
		cur_value = tmp % 10; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} 
/*PORTB=0b00100000;
PORTA=0x00;
_delay_ms(5);
*/

tmp=hours;
do {
		cur_value = tmp % 10; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} while (tmp >= 1);




	PORTB = 0x60; //switch encoder output to unused bit to remove ghosting
}
/********************************************************************/
void Button(){
uint8_t checker =NotTouched-  PINA;
 switch(checker){

	case 0x01:
		edit_hour ^=0x01;
		BARAGRAPH ^=( 1 << 0 );
		break;
    
	case 0x02:
    		edit_minutes ^=0x01;
		BARAGRAPH ^= ( 1 << 1);
	

		break;

	case 0x04:
		alarm ^=0x01;
		BARAGRAPH ^= ( 1 <<2);
	
		break;

	case 0x08:
		alarm_minutes= alarm_minutes+5;
		BARAGRAPH ^= (1<<5);
		break;
}
}
/******************************************encoder************************/
void encoder(){


 OE1 = encoder1;
  OE2 = encoder2;

  

  encoder1 =  (spdr_to_encoder & 0x03);
  encoder2 = ((spdr_to_encoder& 0x0C) >> 2);



/*************regular hour encoder**********************/  
  if(encoder1 != OE1){
    if((OE1 == 0x01) && (encoder1 == 0x03)){
  
	if(edit_hour & edit_minutes){    	
	}
	  	else if (edit_hour){
	    	hours=hours+1;
		}
	  	else if (edit_minutes){
	    	minutes=minutes+1;
		}
  	else{
   
	    	}
	}


    if((OE1 == 0x02) && (encoder1 == 0x03)){      
		if(edit_hour & edit_minutes){    	
		}
	  	else if (edit_hour){
	    	hours=hours-1;
		}
	  	else if (edit_minutes){
	    	minutes=minutes-1;
		}
	  	else{
		}
    	

	}
  }
/********************************************************/
  if(encoder2 != OE2){
    if((OE2 == 0x01) && (encoder2 == 0x03)){     
	OCR3A=OCR3A+50;
	}
    if((OE2 == 0x02) && (encoder2 == 0x03)){
	OCR3A=OCR3A-50;
	}
	}



}
/******************************Clock checker*******************/
void checker(){
if(sec==60){
	sec=0;
	minutes++;

}

if(minutes==60){

	minutes=0;
	hours++;

}

if(hours==13){
	hours=1;

}



if(vol_n <0){
vol_n=0;
}

if( vol_n >2000){
vol_n=2000;
}
/*
if(vol_n==vol_o){}
	else{
	OCR3A=vol_n;
	vol_o=vol_n;
	}
*/


}
/*****************************Check if alarm == clock *********/
void alarm_check(){

if( (hours == alarm_hours) && (minutes==alarm_minutes)){

	alarmON=1;
}

else{

	alarmON=0;
}


}
/**************************LCD control***********************/
void lcd_control(){

if(strcmp(lcd_now,lcd_old)){
	strcpy(lcd_old,lcd_now);
	clear_display();
	string2lcd(lcd_now);


}

}
/*************************************************************TIMER 2**********************/
ISR(TIMER2_OVF_vect){

}

/************************************ADC***********************/
ISR(ADC_vect){

adcr=ADC;
}

/***********************************TIMER 1 **********************/
ISR(TIMER1_COMPA_vect){

PORTD ^=ALARM_PIN;
if(beat >= max_beat) {   //if we've played the note long enough
    notes++;               //move on to the next note
    play_song(song, notes);//and play it
  }

}
/**********************************************************TIMER 0 *************************************/
ISR(TIMER0_OVF_vect){




/**********************************seconds*******************************/
counter++;
if(counter % 8 == 0) {
    //for note duration (64th notes) 
    beat++;
  }  

if((counter %128)==0) { 
//OCR3A=vol_n;

PORTB=decoder_select[4];
PORTA=0xFC;
_delay_ms(100);
sec++;
//ADCSRA |= (1<<ADSC);

}

if((counter %132)==0){
ADCSRA |= (1<<ADSC);

}
/******************************************************************BUTTONS*****************************************************************************/
 /*enabling button to read */
  DDRA = 0x00; //make PORTA as an input
  PORTA = 0xFF; // pull ups
  PORTB=0x70;

  _delay_ms(1); 

 //sinc there is pull ups so the value of PORTA will be deducted from 0xff
  if(PINA != NotTouched){ 
    if(nothing){
        
	Button();
        nothing = 0; 
    }
    else if(PINA == SavedPORTA_Value){ //do nothing
      
    }
    else if(PINA != SavedPORTA_Value){
   Button();
      
	
    }

    SavedPORTA_Value = PINA;
  }
  else {
    nothing = 1;  
  }
 
	DDRA = 0xFF; 
	PORTB =0x70;  
/***********************************************************************finished the button part****************************/


/******************************************start the ISP part, where seding data to SPDR*************************************************************/

	

	PORTE =0x00;
	PORTE =0x80;  
 
	_delay_ms(2);


  SPDR = BARAGRAPH;		//SPDR SEND Data so it can receive, and vice versa 
  spdr_to_encoder = SPDR;

  
  while (bit_is_clear(SPSR, SPIF)){};


			//low high
  PORTE =   0xC0 ;
  PORTE =   0x40;

 
  PORTB =  0x71;
  PORTB = 0x70;

/**************************************************END The ISP Part*********************************************************************************/
  
/****************************************************** ENCODER************************************************************/
  encoder();




}

int main()
{

/************set up ports *******/
  DDRA = 0xFF; 
  DDRB =0xF7;  //enabling the bits from 0 to 2 as well as the upper nibble  
  DDRE = 0xC0;  // enabling bit 6 and 7
  PORTE= 0x40;  //  bit 6 as high for the SPDR and graph
/******************/
/*******TIMER 0 *********/
TIMSK |= (1<<TOIE0); 
/*TCCR0 |= (1<<CS02) | (1<<CS01) | (1<<CS00);*/  //Normal mode but not 128
TCCR0 |= (1<<CS00);
ASSR  |= (1 <<AS0);

 /*while(bit_is_set(ASSR, TCN0UB));
  while(bit_is_set(ASSR, OCR0UB));
  while(bit_is_set(ASSR, TCR0UB));*/
/***********************************TIMER2*****************/
TCCR2|= (1<<WGM21) |(1<< WGM20) | (1<<COM21) |(1<<CS21) |(1<<CS20);

/******ADC Configures****************/

DDRF|=0x01;  //using pin 1 in the portf
PORTF=0x01;

ADMUX= (1<<REFS0); //with external cappacitor, the light sensor one. 

ADCSRA= (1<< ADEN) | (1 <<ADIE) | (1<<ADPS0) | (1<<ADPS1)|(1<<ADPS2); //|(1 <<ADSC);

/******************THE SPDR and BARGRAPH******************************************/
SPCR = (1 << SPE) | (1 << MSTR) ;
/**********************************AUDIO PORT and TIMER1***************************************/
DDRD=0xff;
/*
TCCR1A |=0x00;
TCCR1B |= (1 <<WGM12) | ( 1<< CS10);
TCCR1C |=0x00;
//OCR1A   =20000;
TIMSK |= (1<<OCIE1A);
*/

music_init();
//OCR1A   =30000;
/**************************TIMER 3 VOlume ********************/
DDRE|=0x08;

TCCR3A |= (1<< COM3A1) |(1<<WGM31);
TCCR3B |= (1 <<WGM32) | (1 <<CS31);
TCCR3C|=0x00;

OCR3A=1000;

//vol_n=1000;
/***********************************************************/
	lcd_init();

	clear_display();
	string2lcd("Mohammed");
	//_delay_ms(1000);	
	sei(); 
 	DDRA = 0xFF; 
	PORTB =0x70;
//	music_on();
//	checker();
		
  while(1){  
	//OCR2=250;
	//music_on();
/*******displying part*********/

	
	if (alarm){
		display_alarm();
		strcpy(lcd_now,"set alarm");	
		
		
	}

	else if(alarmON){
		display_digits();
		strcpy(lcd_now,"Wake UP");
		vol_n=700;
		if(fix==0){
		music_on();
		fix=1;		
		//OCR1A= 10000;
		}
	}
	else{
		display_digits();
		strcpy(lcd_now,"           ");
		//strcpy(lcd_now,adcr);
		//OCR1A=0;
		
		music_off();
		fix=0;
	}	
	

/******************************/
	adcr=ADC;
	//itoa(adcr,lcd_now,10);
	if(adcr <150){
		OCR2=0;
	}
	else{OCR2=210;}
	

	
 	//OCR1A   =40000;
	lcd_control();

	alarm_check();
	checker();

  }//while
}//main

/*ECE 473, lab wrriten by Mohammed Alarifi*/

#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include"music.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "si4734.h"

extern uint16_t current_fm_freq;
volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done

ISR(INT7_vect){  STC_interrupt = TRUE;  }

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
uint8_t temp=0;
uint8_t temp2=0;

uint8_t radio=0;
fix2=0;


/**************************************lAB 5 SUFF ***********************/
uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[5];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number
char    lcd_string_array[16];  //holds a string to refresh the LCD
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result;  
char     lcd_str_h[5];  //holds string to send to lcd  
char     lcd_str_l[5];  //holds string to send to lcd  
extern uint8_t lm73_wr_buf[2]; 
extern uint8_t lm73_rd_buf[2];
int16_t lm73_temp;

void spi_init(void){
  DDRB   = DDRB | 0x07;           //Turn on SS, MOSI, SCLK pins
  SPCR  |= (1<<SPE) | (1<<MSTR);  //set up SPI mode
  SPSR  |= (1<<SPI2X);            //run at double speed 
}//spi_init    

/*************************Test function**********************/
double get_48(){

clear_display();
	 
	string2lcd("a");	
	string2lcd(lcd_str_array);	  
        rcv_rdy=0;
        //cursor_home();



}
/*******************************/
double get_128(){

//_delay_ms(50); //tenth second wait                  //wipe the display
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);//read temperature data from LM73 (2 bytes) 
  //_delay_ms(2);    //wait for it to finish
  lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  lm73_temp = lm73_temp << 8; //shift it into upper byte 
  lm73_temp |= lm73_rd_buf[1];  //"OR" in the low temp byte to lm73_temp 
  //lm73_temp =
  //itoa(lm73_temp, lcd_string_array, 10); //convert to string in array with itoa() from avr-libc                           
  //string2lcd(lcd_string_array); //send the string to LCD (lcd_functions)
  fp_adc_result = div(lm73_temp, 128);              //do division by 205 (204.8 to be exact)
  itoa(fp_adc_result.quot, lcd_str_h, 10);           //convert non-fractional part to ascii string
  fp_low_result = div((fp_adc_result.rem*100), 128); //get the decimal fraction into non-fractional form 
  itoa(fp_low_result.quot, lcd_str_l, 10);           //convert fractional part to ascii string



}

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
/***/



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
/*********************/
void display_freq(){
cur_digit=0;

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups

tmp=current_fm_freq;
for(i=0; i<4 ; i++){
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





	PORTB = 0x60; //switch encoder output to unused bit to remove ghosting





}
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
		BARAGRAPH ^= (1<<3);
		break;

	case 0x10:
		temp ^=0x01;
		BARAGRAPH ^= (1<<4);
		break;



	case 0x20:
		temp2 ^=0x01;
		BARAGRAPH ^= (1<<5);
		break;


	case 0x40:
		minutes=minutes+1;

		break;

	case 0x80:
		
		radio ^=0x01;
		BARAGRAPH ^= (1<<7);
		
    		
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
if (temp==0){
if(strcmp(lcd_now,lcd_old)){
	strcpy(lcd_old,lcd_now);
	clear_display();
	string2lcd(lcd_now);


}
}

	else{
		if(temp2==0){


			strcpy(lcd_now,lcd_str_h);
			if (strcmp(lcd_now,lcd_old)){
				strcpy(lcd_old,lcd_now);
				clear_display();
				string2lcd(lcd_now);
				char2lcd('.');
				string2lcd(lcd_str_l);
			}	
		}



		else{
			if(rcv_rdy==1){
				
					strcpy(lcd_now,lcd_str_array);
					if (strcmp(lcd_now,lcd_old)){
						strcpy(lcd_old,lcd_now);
						clear_display();
						string2lcd(lcd_now);
					}
					
					  
			rcv_rdy=0;


			}



		}
	


	}

}
/*************************************************************TIMER 2**********************/
ISR(TIMER2_OVF_vect){

}
/****************lab 5*****************/
ISR(USART0_RX_vect){
static  uint8_t  i;
  rx_char = UDR0;              //get character
  lcd_str_array[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    lcd_str_array[--i]  = (' ');     //clear the count field
    lcd_str_array[i+1]  = (' ');
    lcd_str_array[i+2]  = (' ');
    i=0;  
  }

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
get_128();

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
/*******************part 5 ******************/
 DDRF |= 0x08; //lcd strobe bit
  uart_init();  
  spi_init();
 // lcd_init();
  clear_display();
  cursor_home();
  init_twi();
/****/

DDRE  |= 0x08;
    PORTE |= 0x08;
    
    DDRE  |= 0x04; //Port E bit 2 is active high reset for radio
    PORTE |= 0x04; //radio reset is on at powerup (active high)
    
    EICRB |= (1<<ISC71) | (1<ISC70);
    EIMSK |= (1<<INT7);
    
    //hardware reset of Si4734
    PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
    DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
    PORTE |=  (1<<PE2); //hardware reset Si4734
    _delay_us(200);     //hold for 200us, 100us by spec
    PORTE &= ~(1<<PE2); //release reset
    _delay_us(30);      //5us required because of my slow I2C translators I suspect
    //Si code in "low" has 30us delay...no explaination
    DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

/****/
 
 
lm73_wr_buf[0] = 0x90; //load lm73_wr_buf[0] with temperature pointer address
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);//start the TWI write process
_delay_ms(2);    //wait for the xfer to finish
//clear_display(); //clean up the display

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
//	 sei();


    
    sei();

   // fm_pwr_up(); //powerup the radio as appropriate
   // current_fm_freq = 9990;
    //fm_tune_freq();

		
  while(1){  
		if(radio==0x01 & fix2==0){
			fm_pwr_up(); //powerup the radio as appropriate

    			current_fm_freq = 9990;

    			fm_tune_freq();
			fix2=1;
		}

		if (radio==0x00){
			//radio_pwr_dwn();
			fix2=0;
		}		

	
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
	
	else if( radio==0x01){

		display_freq();

	}
	else{
		display_digits();
		strcpy(lcd_now,"           ");
		
		
		music_off();
		fix=0;
	}	
	

	lcd_control();

	alarm_check();
	checker();

  }//while
}//main








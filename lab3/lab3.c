/*ECE 473, lab wrriten by Mohammed Alarifi*/

#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>

uint16_t counter = 0;
uint16_t bit0 = 0x00; //if mode 2 , it called bit because the way it displayed in the bargraph
uint16_t bit1 = 0x00;//for mode 4
uint16_t  SavedPORTA_Value = 0;
uint16_t NotTouched=0xFF;
uint16_t  nothing = 1;
uint16_t  spdr_to_encoder;
uint16_t  BARAGRAPH = 0;
uint16_t  OE1 = 0;//old value of encoder 1
uint16_t  OE2 = 0; //old value of encoder 2
uint16_t  encoder1 = 0;
uint16_t  encoder2 = 0;
uint16_t hex=0;
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
	0b00010000, //center colon
	0b01110000  //hi-Z mode
};
/****************/
void display_digits() 
{
	uint16_t tmp = counter; //tmp variable to modify number for display
	uint8_t cur_value; //current digit value to display
	uint8_t cur_digit = 0; //current digit to display on

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups


	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
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
void display_digits_hex() 
{
	uint16_t tmp = counter; //tmp variable to modify number for display
	uint8_t cur_value; //current digit value to display
	uint8_t cur_digit = 0; //current digit to display on

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups
	

	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
	do {

	


		cur_value = tmp % 16; //get current digit to display
		PORTB=0x00;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF; //clear PORTA
		PORTA = sev_seg_digits_hex[cur_value]; //display digit
		_delay_ms(5);
		cur_digit++; //next digit
		tmp /= 16; //get next value
	} while (tmp >= 1);
	PORTB = 0x20;
	PORTA = 0b11111011;
	_delay_ms(5);
	PORTB = 0x60; //switch encoder output to unused bit to remove ghosting
}


/**************************/



ISR(TIMER0_OVF_vect){


  DDRA = 0x00; //make PORTA as an input
  PORTA = 0xFF; // pull ups
  PORTB=0x70;

  _delay_ms(1); 

 //sinc there is pull ups so the value of PORTA will be deducted from 0xff
  if(PINA != NotTouched){ 
    if(nothing){
      /***************************/
	uint8_t checker =NotTouched-  PINA;
  switch(checker){

	
case 0x04:
if(hex==0x00){
hex=0x01;

}

else{ hex=0x00;}
break;


    case 0x01:
	if(bit0==0x00){
		bit0=0x01;
		if (BARAGRAPH==0x00){
			BARAGRAPH=0x01;
		}
		else{
			BARAGRAPH=0x03;
		}
	}
	else{
		bit0=0x00;
		if (BARAGRAPH==0x01){
			BARAGRAPH=0x00;
		}
		else{
			BARAGRAPH=0x02;
		}
	}
	
      break;
    
case 0x02:
    if(bit1==0x00){
		bit1=0x01;
		if (BARAGRAPH==0x00){
			BARAGRAPH=0x02;
		}
		else{
			BARAGRAPH=0x03;
		}		
	}
    else{
     		 bit1 = 0x00;
		if (BARAGRAPH==0x02){
			BARAGRAPH=0x00;
		}
		else{
			BARAGRAPH=0x01;
		}
		
		}


  }
	
        nothing = 0; 
    }
    else if(PINA == SavedPORTA_Value){ //do nothing
      
    }
    else if(PINA != SavedPORTA_Value){
   
      		uint8_t checker =NotTouched-  PINA;
  switch(checker){
    

case 0x04:
if(hex==0x00){
hex=0x01;

}

else{ hex=0x00;}
break;


case 0x01:
	if(bit0==0x00){
		bit0=0x01;
		if (BARAGRAPH==0x00){
			BARAGRAPH=0x01;
		}
		else{
			BARAGRAPH=0x03;
		}
	}
	else{
		bit0=0x00;
		if (BARAGRAPH==0x01){
			BARAGRAPH=0x00;
		}
		else{
			BARAGRAPH=0x02;
		}
	}
	
      break;
    
case 0x02:
    if(bit1==0x00){
		bit1=0x01;
		if (BARAGRAPH==0x00){
			BARAGRAPH=0x02;
		}
		else{
			BARAGRAPH=0x03;
		}		
	}
    else{
     		 bit1 = 0x00;
		if (BARAGRAPH==0x02){
			BARAGRAPH=0x00;
		}
		else{
			BARAGRAPH=0x01;
		}
		
		}

  }




	
    }





    SavedPORTA_Value = PINA;
  }
  else {
    nothing = 1;  
  }

 
	DDRA = 0xFF; 
 
	PORTB =0x70;  


	

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

/***************************************************************************************************************************************************/
  
  OE1 = encoder1;
  OE2 = encoder2;

  

  encoder1 =  (spdr_to_encoder & 0x03);
  encoder2 = ((spdr_to_encoder& 0x0C) >> 2);

  
  if(encoder1 != OE1){
    if((OE1 == 0x01) && (encoder1 == 0x03)){
  
	if(bit0 & bit1){    	
	}
  	else if (bit0)
    	counter =counter+ 2;
  	else if (bit1)
    	counter =counter+ 4;
  	else
    	counter = counter+1;
    
	}
    if((OE1 == 0x02) && (encoder1 == 0x03)){      
	if(bit0 & bit1){    	
	}
  	else if (bit0)
    	counter =counter- 2;
  	else if (bit1)
    	counter =counter- 4;
  	else
    	counter =counter- 1;

	}
  }
  if(encoder2 != OE2){
    if((OE2 == 0x01) && (encoder2 == 0x03)){     
	if(bit0 & bit1){
	}
  	else if (bit0)
    	counter = counter+2;
  	else if (bit1)
    	counter =counter+ 4;
  	else
    	counter= counter+1;
	}
    if((OE2 == 0x02) && (encoder2 == 0x03)){
	if(bit0 & bit1){
	
	}
	else if (bit0)
    	counter =counter- 2;
  	else if (bit1)
    	counter = counter-4;
  	else
    	counter = counter-1;
	}
	}
}

int main()
{

/*******************/
  DDRA = 0xFF; 
  DDRB =0xF7;  //enabling the bits from 0 to 2 as well as the upper nibble  
  DDRE = 0xC0;  // enabling bit 6 and 7
  PORTE= 0x40;  //  bit 6 as high
/******************/
TIMSK |= (1<<TOIE0); 
TCCR0 |= (1<<CS02) | (1<<CS01) | (0<<CS00);  //Normal mode but not 128
/**********************/
SPCR = (1 << SPE) | (1 << MSTR) ;
/***************/
	sei(); 
 	DDRA = 0xFF; 
	PORTB =0x70;
  while(1){  

if (hex==0x01){
display_digits_hex();
}
else if(hex==0x00){
display_digits();
}

  }//while
}//main

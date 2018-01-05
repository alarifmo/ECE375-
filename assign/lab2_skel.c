// lab2_skel.c 
// Mohammed Alarifi


//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include<stdio.h>
#include<stdlib.h>


//decimal to 7-segment LED display encodings, The idea from ECE 375 each sapce will define the encoding for the that number in the 7 segment
int current_number=0; // hold the number displayed in the 7 segmant
			// I put the varible here so i can use it in the check function with no need to pass it


uint8_t dec_to_7seg[10]= {
	0xC0,// FOR 0
	0xF9,//== 1
	0xA4,//== 2
	0xB0,//==3
	0x99,//==4
	0x92,//==5
	0x83,//==6
	0xF8,//==7
	0x80,//==8
	0x98 //==9
};
		
//***************************************************************************** 
//Debounsing check from the first lab, with the edit to make it check for each button, depending on the button number.
int8_t debounce_switch(uint8_t i) {
static uint16_t state [8];
  state[i] = (state[i] << 1) | (! bit_is_clear(PINA, i)) | 0xE000;
  if (state[i]== 0xF000) return 1;
  return 0;
}
/**************check if we reached 1023******************/
void check(){

if(current_number >1023){
	current_number=1;
	}

}

//***********************************************************************************
uint8_t main()
{




DDRA=0xff; //Make PORTA A all output, because i want to disply the 0 first
DDRB=0xF0;// Set port bit as output 4-7 bits
PORTB=0x70; // to select the zero digit, so we eanble selecters
PORTA=0xFF; // to display 0





while(1){	
/*the skelton code suggest to check for button here, but i will send the value to the display first so i can display a 0 there before anything */


/*Get copy of the current number so we can minuplate it and get its reminder, the logic is explained at the bottom*/

int current_number_copy = current_number;	//copy the value so we can minuplate it
int digit=0;				//we will start from the zero digit

/*change register A to be output */
DDRA=0xFF;
DDRB=0xF0;
PORTB=0x00;
PORTA=0xFF;

/*The logic for getting the transforming the number to the 4-digit-7 segment display. is from the internet. I got the idea and i implement it my self*/
/* The idea is to get the reminder of the current number and save it in the first digit, then divide the intial number by 10, then get the reminder and save it in the seocnd digit, and so on. You need to stop if the current number become less than 1, that mean there are no more digit */
_delay_ms(1);

while (current_number_copy >=1) {		//the cope is less than 1 then it mean we displayed all the numbers
if(digit==0){					//if digit is 0 it mean we are in the zero digit	
PORTB=0x00;					//Goto digit zero
PORTA=0xFF;					//if i dont do this, sometimes there is lite light from the prevoius number
PORTA= dec_to_7seg[current_number_copy % 10];	//Here is when we get the least number to the right, and from the array we know its encoding
current_number_copy=current_number_copy/10; //divide the number by 10 to get rid of the number we just put in digit zero
}
else if(digit==1){		//All the rest code follow the same logic as the prevoius if statment

PORTB=0x00;
PORTB= 0x10;
PORTA=0xFF;
PORTA= dec_to_7seg[current_number_copy % 10];
current_number_copy=current_number_copy/10; 
}
else if(digit==2){

PORTB=0x00;
PORTB=0x30;
PORTA=0xFF;
PORTA= dec_to_7seg[current_number_copy % 10];
current_number_copy=current_number_copy/10; 
}
else if(digit==3){

PORTB=0x00;
PORTB= 0x40;
PORTA=0xFF;
PORTA= dec_to_7seg[current_number_copy % 10];
current_number_copy=current_number_copy/10; 

}

digit=digit+1;
_delay_ms(1);


}
/********************************** Switch check **************************/
        PORTB=0x70; // enable selcters
	DDRA = 0x00; //Make PORTA input
	PORTA = 0xFF; //enabling the Pull ups for PORTA


	int i;
	for (i=0; i<8 ;i++){					//for loop to help with the if statemtns which check each button through
								//the function provided in the lab 1
	if (debounce_switch(i)){
		if(i==0){		

		current_number =current_number+1;
		}
		else if (i==1){		
		current_number =current_number+2;		
	}
		else if(i==2){		
		current_number =current_number+4;
	}
		else if(i==3){		
		current_number =current_number+8;
	}
		else if(i==4){		
		current_number =current_number+16;
	}
		else if(i==5){		
		current_number =current_number+32;
	}
		else if(i==6){		
		current_number =current_number+64;
	}
		else if(i==7){		
		current_number =current_number+128;
	}

}
}


check();				//check if the number is higher than 1023 then make it equal to 1







  }//while
}//main

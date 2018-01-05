//heartint.c
//setup TCNT1 in pwm mode, TCNT3 in normal mode 
//set OC1A (PB5) as pwm output 
//pwm frequency:  (16,000,000)/(1 * (61440 + 1)) = 260hz
//
//Timer TCNT3 is set to interrupt the processor at a rate of 30 times a second.
//When the interrupt occurs, the ISR for TCNTR3 changes the duty cycle of timer 
//TCNT1 to affect the brightness of the LED connected to pin PORTB bit 5.
//
//to download: 
//wget http://www.ece.orst.edu/~traylor/ece473/inclass_exercises/timers_and_counters/heartint_skeleton.c
//to print: a2ps -P <printer> -1 --font-size=9 heartint_skeleton.c

#include <avr/io.h>
#include <avr/interrupt.h>

#define TRUE  1
#define FALSE 0

//Traverse the array up then down with control statements or double the size of
//the array and make the control easier.  Values from 0x0100 thru 0xEF00 work 
//well for setting the brightness level.

uint16_t brightness[10] = { 0x0100, 0xA100,0xB100,0xC1,0xD100,0xE100,0x0F00,0x0A00,0x0B00,0x0C00                                                } ;

ISR(TIMER3_OVF_vect                      ) {
uint16_t time_tick;                                    
                                      
time_tick++;                   
                            

switch(time_tick){


case 10: OCR1A =brightness[1]; break;
case 20: OCR1A =brightness[3]; break;
case 30: OCR1A =brightness[5]; break;
case 40: OCR1A =brightness[7]; break;








}

}

int main() {

  DDRB    =0x10;                             //set port B bit five to output

//setup timer counter 1 as the pwm source

  TCCR1A |=     (0<<WGM10) |(1<<WGM11)|(1<<COM1C0)|(1<<COM1C1);                        //fast pwm, set on match, clear@bottom, 
                                        //(inverting mode) ICR1 holds TOP

  TCCR1B |=    (0<<CS10)|(0<<CS11)|(0<<CS12);                         //use ICR1 as source for TOP, use clk/1

  TCCR1C  =    (0<<FOC1A)|(0<<FOC1A)|(0<<FOC1A);                         //no forced compare 

  ICR1    =       0xF000;                      //clear at 0xF000                               

  
//setup timer counter 3 as the interrupt source, 30 interrupts/sec
// (16,000,000)/(8 * 2^16) = 30 cycles/sec

  TCCR3A =     (0<WGM30)|(0<<WGM31)|(0<<COM3C0)|(0<<COM3C1);                         //normal mode

  TCCR3B =      (0<<CS30)|(1<<CS31)|(0<<CS32);                        //use clk/8  (15hz)  

  TCCR3C =      (0<<FOC3A)|(0<<FOC3A)|(0<<FOC3A);            //no forced compare 

  ETIMSK =    0xFF;                          //enable timer 3 interrupt on TOV

  sei();                                //set GIE to enable interrupts
  while(1) { } //do forever
 
}  // main

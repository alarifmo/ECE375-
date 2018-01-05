/*********************************************************************/
/*                   Music functions for ATMEGA128                   */
/*                        Kellen Arb 10.18.08                        */
/* This file should include everything you need to set up and use    */
/*songs for your ECE473 alarm clock.  Each function has a description*/
/*so you know how to use it, but all you should need to do is:       */
/*  0)Add the following to your Timer0 overflow interrupt (assuming  */
/*      interrupt is 128 times/second):
  ms++;
  if(ms % 8 == 0) {
    //for note duration (64th notes) 
    beat++;
  }                                                                  */
/*      ms can be changed for any counter variable, if you have one  */
/*      already. beat, however, must stay.                           */
/*  1)Change the #define values below for mute, unmute, and ALARM_PIN*/
/*      to the values needed for your setup.  If you use a different */
/*      port, as well as different pins, you'll have to manually     */
/*      change them throughout this file.                            */
/*  2)In your main function, call music_init().  Check to make sure  */
/*      there aren't any conflicts with the values it sets.          */
/*  4)Set the value of "song" to your liking. You can make this user */
/*      selectable very easily.                                      */
/*  5)Anytime you set off your alarm, add a call to music_on(). This */
/*      will start the interrupt and the song playing.               */
/*  4)Anytime you turn off your alarm, add a call to music_off().    */
/*      this stops the song playing and halts the interrupt.         */
/*                                                                   */
/* That should be all you need!  If you want to create new songs,    */
/*just read the descriptions for play_note and play_rest and copy the*/
/*format of any of the pre-done songs.  Make sure to share any good  */
/*ones with the class! Have fun!                                     */
/*             -Kellen Arb                                           */
/*********************************************************************/
#include <avr/io.h>
#define F_CPU 16000000UL //16Mhz clock
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>
#include "si4734.h"
#include "uart_functions.h"


#include "twi_master.h" //my defines for TWCR_START, STOP, RACK, RNACK, SEND

volatile uint16_t  current_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
extern uint8_t  si4734_wr_buf[9];
extern uint8_t  si4734_rd_buf[9];
extern uint8_t  si4734_tune_status_buf[8];
extern volatile uint8_t STC_interrupt;     //indicates tune or seek is done
uint8_t rssi ;



uint8_t si4734_revision_buf[16];   //buffer for holding revision  data  

enum radio_band{FM, AM, SW};
extern volatile enum radio_band current_radio_band;

volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done

extern uint16_t eeprom_fm_freq;
extern uint16_t eeprom_am_freq;
extern uint16_t eeprom_sw_freq;
extern uint8_t  eeprom_volume;
extern uint16_t current_radio_band;

//extern uint16_t current_fm_freq;
extern uint16_t current_am_freq;
extern uint16_t current_sw_freq;
extern uint8_t  current_volume;

//Used in debug mode for UART1
extern char uart1_tx_buf[40];      //holds string to send to crt
extern char uart1_rx_buf[40];      //holds string that recieves data from uart
ISR(INT7_vect){STC_interrupt = TRUE;}

//Mute is on PORTD
//set the hex values to set and unset the mute pin
//I used PORTD-PIN2
#define mute 0x04
#define unmute 0xFB
//Alarm is also on PORTD
//set the hex value for the alarm pin
//I used PORTD-PIN7
#define ALARM_PIN 0x80
#define NUM_SONGS 4
//set this variable to select the song
//(0-3, unless you add more)
volatile uint8_t song;
uint8_t counter; 

void music_off(void);
void music_on(void);      
void music_init(void);




/*********************************************************************/
/*                             TIMER1_COMPA                          */
/*Oscillates pin7, PORTD for alarm tone output                       */
/*********************************************************************/




/*
ISR(TIMER1_COMPA_vect) {
  PORTD ^= ALARM_PIN;      //flips the bit, creating a tone
  if(beat >= max_beat) {   //if we've played the note long enough
    notes++;               //move on to the next note
    play_song(song, notes);//and play it
  }
}
*/

int main(){
TIMSK |= (1<<TOIE0); 
TCCR0 |= (0<<CS02) | (0<<CS01) | (1<<CS00);  //Normal mode but not 128
//TCCR0 |= (1<<CS00);
ASSR  |= (1 <<AS0);

DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
DDRE  |= 0x40; //Port E bit 6 is shift/load_n for encoder 74HC165
DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
PORTE |= 0x04; //radio reset is on at powerup (active high)
PORTE |= 0x40; //pulse low to load switch values, else its in shift mode


DDRD = 0x80;
//DDRE|=0x08;
TCCR3A |= (1<< COM3A1) |(1<<WGM31);
TCCR3B |= (1 <<WGM32) | (1 <<CS31);
TCCR3C|=0x00;
OCR3A=1000;

sei();

PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
PORTE |=  (1<<PE2); //hardware reset Si4734 
_delay_us(200);     //hold for 200us, 100us by spec         
PORTE &= ~(1<<PE2); //release reset 
_delay_us(30);      //5us required because of my slow I2C translators I suspect
                    //Si code in "low" has 30us delay...no explaination
DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
while(1){

//music_on();
fm_pwr_up();        //power up radio
while(twi_busy()){} //spin while TWI is busy 
fm_tune_freq();     //tune to frequency      

//retrive the receive strength and display on the bargraph display
while(twi_busy()){}                //spin while TWI is busy 
fm_rsq_status();                   //get status of radio tuning operation
rssi =  si4734_tune_status_buf[4]; //get tune status 
//redefine rssi to be a thermometer code
  if(rssi<= 8) {rssi = 0x00;} else
  if(rssi<=16) {rssi = 0x01;} else
  if(rssi<=24) {rssi = 0x03;} else
  if(rssi<=32) {rssi = 0x07;} else
  if(rssi<=40) {rssi = 0x0F;} else
  if(rssi<=48) {rssi = 0x1F;} else
  if(rssi<=56) {rssi = 0x3F;} else
  if(rssi<=64) {rssi = 0x7F;} else
  if(rssi>=64) {rssi = 0xFF;}





}
}


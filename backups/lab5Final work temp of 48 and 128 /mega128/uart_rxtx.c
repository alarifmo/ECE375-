//UART Example for inclass coding
//Roger Traylor 12.4.12
//Connect two mega128 boards via rs232 and they should end to each
//other a message and a sequence number.
//
//Change the message you send to your partner for checkoff.
//
//You can test this code by a "loopback" if you connect rx to tx
//on the DB9 connector.
//

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "twi_master.h"

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
	 
	//char f[32];
	//strcpy(f,lcd_str_array);
	//strcpy(f,lcd_str_h);
	//strcpy(f,'.');
	//strcpy(f,lcd_str_l);
	//string2lcd(" wld al3m");
	//string2lcd(" almhay6y");
	string2lcd("a");	
	string2lcd(lcd_str_array);
	line2_col1();  
	//_delay_us(50);
	string2lcd("h");
	string2lcd(lcd_str_h);
	char2lcd('.');          
  	string2lcd(lcd_str_l);

	/*int z,j;
	z= strlen(s1);
	for(j=0; lcd_str_array[j] !='\0'; z++,j++){
	s1[z]=lcd_str_array[j];
	}
	s1[z]='\0';
	
	//char *result= (char *)malloc (1+strlen(s1)+strlen(lcd_str_array));
	//strcpy(result,s1);
	//strcpy(result,lcd_str_array);
	//string2lcd(lcd_str_array);   
	_delay_ms(20);   
	//string2lcd(lcd_str_array);  //write out string if its ready
	string2lcd("H:");
	_delay_ms(50); 
	string2lcd(lcd_str_h);  //write upper half
  	char2lcd('.');          //write decimal point
  	 //string2lcd(lcd_str_l);  //write lower half
	*/  
        rcv_rdy=0;
        cursor_home();



}
/*******************************/
double get_128(){

_delay_ms(50); //tenth second wait
  clear_display();                  //wipe the display
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 4);//read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
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
int main(){
 // int16_t lm73_temp; 
  DDRF |= 0x08; //lcd strobe bit
  uart_init();  
  spi_init();
  lcd_init();
  clear_display();
  cursor_home();
  init_twi();

  sei();
 
lm73_wr_buf[0] = 0x00; //load lm73_wr_buf[0] with temperature pointer address
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);//start the TWI write process
_delay_ms(2);    //wait for the xfer to finish
clear_display(); //clean up the display

 while(1){

	get_128();

//**************  start rcv portion ***************
      if(rcv_rdy==1){
	get_48();

	
    }//if 
//**************  end rcv portion ***************

//**************  start tx portion ***************

  
    /*uart_puts("dksjhfkjsdh");
    itoa(send_seq,lcd_string,10);
    uart_puts(lcd_string);
    uart_putc('\0');
    for(i=0;i<=10;i++){_delay_ms(100);}
    send_seq++;
    send_seq=(send_seq);
*/


//**************  end tx portion ***************
  }//while
}//main

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
//************************************//


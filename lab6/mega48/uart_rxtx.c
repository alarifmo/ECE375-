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
//
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m48.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "twi_master.h"

uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number
extern uint8_t lm73_wr_buf[2]; 
extern uint8_t lm73_rd_buf[2]; 
char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd  
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result;  


void spi_init(void){
  DDRB   = DDRB | 0x07;           //Turn on SS, MOSI, SCLK pins
  SPCR  |= (1<<SPE) | (1<<MSTR);  //set up SPI mode
  SPSR  |= (1<<SPI2X);            //run at double speed 
}//spi_init    

int main(){
  //DDRF |= 0x08; //lcd strobe bit
 int16_t lm73_temp;  //a place to assemble the temperature from the lm73
  uart_init();  
  spi_init();
  init_twi();      //initalize TWI (twi_master.h)  
  sei();

  //set LM73 mode for reading temperature by loading pointer register
lm73_wr_buf[0] = 0x00; //load lm73_wr_buf[0] with temperature pointer address
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);//start the TWI write process
_delay_ms(2);    //wait for the xfer to finish
  while(1){
//**************  start rcv portion ***************
      if(rcv_rdy==1){
  }
//**************  end rcv portion ***************
 twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 4);//read temperature data from LM73 (2 bytes)  
  _delay_ms(2);    //wait for it to finish
lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  lm73_temp = lm73_temp << 8; //shift it into upper byte 
  lm73_temp |= lm73_rd_buf[1];  //"OR" in the low temp byte to lm73_temp 
  fp_adc_result = div(lm73_temp, 128);              //do division by 205 (204.8 to be exact)
  itoa(fp_adc_result.quot, lcd_str_h, 10);           //convert non-fractional part to ascii string
  fp_low_result = div((fp_adc_result.rem*100), 128); //get the decimal fraction into non-fractional form 
  itoa(fp_low_result.quot, lcd_str_l, 10);           //convert fractional part to ascii string
//**************  start tx portion ***************



  
 uart_puts(lcd_str_h);
 uart_puts(".");
 uart_puts(lcd_str_l);

    //itoa(send_seq,lcd_string,10);
    //uart_puts(lcd_string);
    uart_putc('\0');
    //for(i=0;i<=10;i++){_delay_ms(100);}
    //send_seq++;
    //send_seq=(send_seq);



//**************  end tx portion ***************
  }//while
}//main

ISR(USART0_RX_vect){

}
//************************************//


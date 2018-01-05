

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>



ISR(TIMER0_OVF_vect){


}

void configureSPI( void ){
 
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);   

}


//Main function call
int main()
{
SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA); 
sei();
//set port bits 4-7 B as outputs
while(1){
 
  //configureSPI();
  


 
}

}

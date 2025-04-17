/******************************************************************************/
// Title: DAC_MCP4921.c
// Date: Jan 8, 2025
// Program to:
// - Configure DAC MCP4921 using SPI
// - Output analog square wave using sleep function
/*******************************************************************************
********************************************************************************

						  -----------------
			  UART0_TX<->|GPO    USB   VBUS|
		      UART0_TX<->|GP1          VBUS|
		                 |GND           GND|    
						 |GP2        3V3_EN| 
						 |GP3       3V3_OUT|<-->VDD_MCP4921, VREFA_MCP4921   
		                 |GP4      ADC_VREF|                           
						 |GP5          ADC2|                    
						 |GND           GND|<-->-LDAC()
				         |GP6          ADC1|                             
			             |GP7          ADC0|                     
						 |GP8           RUN|                                     
			             |GP9          GP22|                       
						 |GND           GND|
						 |GP10         GP21|                      
			             |GP11         GP21|                                  
			             |GP12         GP19|<-->MOSI
						 |GP13         GP18|<-->SCK
						 |GND           GND| 
						 |GP14         GP17|<-->CS
						 |GP15         GP16|<-->MISO            
						  -----------------   
					Raspberry PI ZERO GPIO Header

//-LDAC is tied low. Write commands will be latched directly into the output latch when a valid 16 clock transmission 
has been received and CS raised.
*******************************************************************************/


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
int high_value = 0;
int a = 12288; //binary 0011 0000 0000 0000 - set the config bits
int b = 4095 ; // binary 0000 1111 1111 1111 - set data to high
int c = 0; // bitwise OR operation betwen config bit and data

void write_data(uint8_t* data) {
    gpio_put(PIN_CS, 0); // Indicate beginning of communication
    spi_write_blocking(SPI_PORT, data, 2); // Send data[]
    gpio_put(PIN_CS, 1); // Signal end of communication
    sleep_ms(100);
}


bool repeating_timer_callback(struct repeating_timer *t) {
    high_value = 1 - high_value;
    if(high_value == 1) {
        a = 12288;
        b = 4095;
    } else {
        b = 0;
    }
    c = a | b;
    uint8_t data[2]; // Array to store 2 bytes of data to be sent
    data[0] = (c >> 8) & 0xFF; //least significant 8 bits
    data[1] =  c & 0xFF;       //most significant 8 bits
    //uint8_t data[2]; // Array to store data to be sent
    //data[0] = 0x30 | 0x0F; // Remove first bit to indicate write operation
    //data[1] = 0xFF; // Data to be sent
    write_data(data);
}

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi
    struct repeating_timer timer;
    sleep_ms(3000);
    add_repeating_timer_ms(2000, repeating_timer_callback, NULL, &timer);

    while (true) {
    //     // Write Operation Example! Set oversampling and power on chip
    //     int a = 12288; //binary 0011 0000 0000 0000 - set the config bits
    //     int b = 4095 ; // binary 0000 1111 1111 1111 - set data to high
    //     int c = a | b; // bitwise OR operation betwen config bit and data
    //     uint8_t data[2]; // Array to store 2 bytes of data to be sent
    //     data[0] = (c >> 8) & 0xFF; //least significant 8 bits
    //     data[1] =  c & 0xFF;       //most significant 8 bits
    //     //uint8_t data[2]; // Array to store data to be sent
    //     //data[0] = 0x30 | 0x0F; // Remove first bit to indicate write operation
    //     //data[1] = 0xFF; // Data to be sent
    //     write_data(data);        
        
    //     //int a = 12288; //binary 0011 0000 0000 0000 - set the config bits
    //     b = 0 ; // binary 0000 0000 0000 0000 - set data to low
    //     c = a | b; // bitwise OR operation betwen config bit and data
    //     //uint8_t data_1[2]; // Array to store data to be sent
    //     data[0] = (c >> 8) & 0xFF; //least significant 8 bits
    //     data[1] =  c & 0xFF;       //most significant 8 bits

    //    // data[0] = 0x30 | 0x00; // Remove first bit to indicate write operation
    //    // data[1] = 0x00; // Data to be sent
    //     write_data(data)
        tight_loop_contents();


    }
}

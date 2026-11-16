/******************************************************************************
 *   Header Includes
 *******************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

#include "eecs_388_lib.h"

/******************************************************************************
 *   Function: setup() - Initializes the Arduino System
 *      Pre condition: 
 *          Hardware must be properly connected (sensors, LEDS, etc.)
 *      Post condition: 
 *          Runs initialization calls one time on power up
 *          Input/Output (IO) pins are configured
 *******************************************************************************/
void setup() 
{

uart_init();

return;
}

/******************************************************************************
 *   Function: loop() - Main execution loop
 *      Pre condition: 
 *          setup() has been executed and system is initialized
 *      Post condition: 
 *          Performs a single iteration of the system's function
 *          Repeates indefinetely unless the board is reset or powered off
 *******************************************************************************/
void loop() 
{
ser_printline("Setup completed.");
ser_write( '\n' );

while( 1 )
    { 
    /* 
        - read a data frame from the TFmini sensor
        
        - from Table 6, we know each LIDAR measurement sends 9 bytes
        - the function `ser_read()` can process 1 byte per function call

        - This if-statement below will read (and check) the first two bytes of a valid data frame
        - You will need to write code to read the subsequent bytes to get the distance data
        - You will need to process the data received from the sensor to get the correct distance value
    */
   if ('Y' == ser_read() && 'Y' == ser_read()) 
        {
            uint16_t dist_L = ser_read();
            uint16_t dist_H = ser_read();
            dist_H = dist_H << 8;
            uint16_t dist = (dist_L | dist_H);

            if(dist <= 60 ){
                gpio_write(GPIO_11, ON);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, OFF);
                delay_ms(100);
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, OFF);
                delay_ms(100);
                ser_printf("%u", dist);
            }

            if((dist <= 100) | (dist > 60)){
                gpio_write(GPIO_11, ON);
                gpio_write(GPIO_12, OFF);
                gpio_write(GPIO_13, OFF);
                ser_printf("%u", dist);
            }
            if((dist <= 200) | (dist > 100)){
                gpio_write(GPIO_11, ON);
                gpio_write(GPIO_12, ON);
                gpio_write(GPIO_13, OFF);
                ser_printf("%u", dist);
            }
            if(dist > 200){
                gpio_write(GPIO_11, OFF);
                gpio_write(GPIO_12, ON);
                gpio_write(GPIO_13, OFF);
                ser_printf("%u", dist);
            }
        }
    }

return;
}
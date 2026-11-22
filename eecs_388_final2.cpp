/******************************************************************************
 *   Header Includes
 *******************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

#include "eecs_388_lib.h"

volatile uint8_t led_state = 0;
volatile uint8_t intct = 0;
volatile uint8_t aidx = 0;
/******************************************************************************
 *   Constant definitions
 *******************************************************************************/

/*ServoMotor constants*/
#define SERVO_PULSE_MAX (2400)    /* 2400 us */
#define SERVO_PULSE_MIN (544)     /* 544 us */
#define SERVO_PERIOD    (20000)   /* 20000 us (20ms) */
#define MIN_ANGLE       (0)       /* degrees */
#define MAX_ANGLE       (180)     /* degrees */

/******************************************************************************
 *   Constant definitions
 *******************************************************************************/
#define BMP180_ADDR     (0x77)    // BMP180 I2C address

#define BASE_CAL_ADDR   (0xAA)    // Base address of calibration data
#define ID_REG_ADDR     (0xD0)    // Chip ID register address+
#define CTRL_MEAS_ADDR  (0xF4)    // Measure control register addresss
#define ADC_MSB_ADDR    (0xF6)    // out_msb ADC register addreess

#define CTRL_TEMP_CMND  (0x2E)    // Measure control register command to start temp conversion

#define BMP180_READ     ((BMP180_ADDR << 1) | 1)    // Read from BMP180 register
#define BMP180_WRITE    ((BMP180_ADDR << 1) | 0)    // Write to BMP180 register

/******************************************************************************
 *   Type declarations
 *******************************************************************************/
typedef struct {
  int16_t  cal_AC1;
  int16_t  cal_AC2;
  int16_t  cal_AC3;
  uint16_t cal_AC4;
  uint16_t cal_AC5;
  uint16_t cal_AC6;
  int16_t  cal_B1;
  int16_t  cal_B2;
  int16_t  cal_MB;
  int16_t  cal_MC;
  int16_t  cal_MD;
} bmp180_calibration;

/******************************************************************************
 *   Global Variables
 *******************************************************************************/
bmp180_calibration calib_data;

/******************************************************************************
 *   Function: get_true_temperature() - Get True Temperature
 *      Pre condition:
 *          None
 *      Post condition:
 *          Takes raw UT value and converts to tenths of a degree celsius
 *******************************************************************************/
static int32_t get_true_temperature( int16_t ut_val )
{

  int32_t x1 = ut_val - calib_data.cal_AC6;
  x1 = x1 * calib_data.cal_AC5;
  x1 = x1 >> 15;

  int32_t x2inta = (int32_t)calib_data.cal_MC << 11;
  int32_t x2intb = x1 + (int32_t)calib_data.cal_MD;
  int32_t x2 = x2inta / x2intb;

  int32_t b5 = x1 + x2;
  int32_t t = (b5 + 8) >> 4;

  return t;

/* Task 1.3 - Use the calib_data values to calculate the tempeature from the ut value */
}

/******************************************************************************
 *   Function: read_calibration() - Read Calibration
 *      Pre condition:
 *          Hardware must be properly connected (BMP180 sensors, etc.)
 *          i2c_init() must have been run
 *      Post condition:
 *          Reads the 11 16-bit calibration data values needed to adjust the
 *            raw temp ADC UT value
 *******************************************************************************/
static void read_calibration( bmp180_calibration * calib)
{
  i2c_start();
  i2c_write_address(BMP180_WRITE);        // BMP180 Address + Write mode
  i2c_write(BASE_CAL_ADDR);               // Start at EEPROM base calibration address
  i2c_start();
  i2c_write_address(BMP180_READ);         // BMP180 Address + Read mode

  calib->cal_AC1 = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // Read 0xAA, 0xAB
  calib->cal_AC2 = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // Read 0xAC, 0xAD
  calib->cal_AC3 = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // Read 0xAE, 0xAF
  calib->cal_AC4 = ((uint16_t)i2c_read_ack() << 8) | i2c_read_ack();  // ...
  calib->cal_AC5 = ((uint16_t)i2c_read_ack() << 8) | i2c_read_ack();  // ...
  calib->cal_AC6 = ((uint16_t)i2c_read_ack() << 8) | i2c_read_ack();  // ...
  calib->cal_B1  = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // ...
  calib->cal_B2  = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // ...
  calib->cal_MB  = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // ...
  calib->cal_MC  = ((int16_t)i2c_read_ack()  << 8) | i2c_read_ack();  // ...
  calib->cal_MD  = ((int16_t)i2c_read_ack()  << 8) | i2c_read_nack(); // Last bytes - Read 0xBE, 0xBF

  i2c_stop();

  // verify values - These should stay the same each power cycle
  ser_printf("AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u\n",
    calib->cal_AC1, calib->cal_AC2, calib->cal_AC3, calib->cal_AC4, calib->cal_AC5, calib->cal_AC6);
  ser_printf("B1=%d B2=%d MB=%d MC=%d MD=%d\n",
    calib->cal_B1, calib->cal_B2, calib->cal_MB, calib->cal_MC, calib->cal_MD);
}
/******************************************************************************
 *   Local function prototypes
 *******************************************************************************/

/******************************************************************************
 *   Function: auto_brake() - Auto Brake
 *      Pre condition: 
 *          None
 *      Post condition: 
 *          Checks the LiDAR distance and configures LEDs
 *******************************************************************************/
static void auto_brake(char c)
{
  if (c == 's') {
    gpio_write(GPIO_13, OFF);
    gpio_write(GPIO_12, ON);
    gpio_write(GPIO_11, OFF);
} //turn on green LED
  if (c == 'c') {
    gpio_write(GPIO_13, ON);
    gpio_write(GPIO_12, ON);
    gpio_write(GPIO_11, OFF);
} //turn on yellow in LED
  if (c == 'v') {
    gpio_write(GPIO_13, ON);
    gpio_write(GPIO_12, OFF);
    gpio_write(GPIO_11, OFF);
} //turn on red LED
	if (c == 't') {
		if (led_state == 0) {
  		gpio_write(GPIO_13, ON);
      gpio_write(GPIO_12, OFF);
      gpio_write(GPIO_11, OFF);
			led_state = 1;
		}
		else {
			gpio_write(GPIO_13, OFF);
      gpio_write(GPIO_12, OFF);
      gpio_write(GPIO_11, OFF);
			led_state = 0;
		}
	} //blink the red LED
}


/******************************************************************************
 *   Function: steering() - Steering
 *      Pre condition: 
 *          None
 *      Post condition: 
 *          Control the servomotor with GPIO_6
 *******************************************************************************/

/******************************************************************************
 *   Function: setup() - Initializes the Arduino System
 *      Pre condition: 
 *          Hardware must be properly connected (BMP180 sensors, etc.)
 *      Post condition: 
 *          Runs initialization calls one time on power up
 *          UART is initialized for ser_printf()
 *          I2C is initialized for communication with BMP180
 *          calib_data is filled with calibration data from BMP180
 *******************************************************************************/
static void servo (int gpio, volatile int angle) {
    //do lerp to determine pulse width based on angle, use uint32 because it can store 2^32 us/ 4294 seconds
	volatile uint32_t on_us = SERVO_PULSE_MIN + (((SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180) * angle);
	gpio_write(gpio, ON);
	delay_us(on_us);

	gpio_write(gpio, OFF);
	uint32_t servo_period_ms = SERVO_PERIOD / 1000;
	uint32_t off_ms = servo_period_ms - (on_us / 1000);
	uint32_t off_us = on_us % 1000;
	delay_ms(off_ms);
	delay_us(off_us);
// YOUR CODE HERE
// Basically, you need to take the input angle "angle" and generate the corresponding pwm signal
// To generate the pwm signals, use gpio_write(), delay_us(), and delay_ms().

return;
}

static void steering(int gpio, int angle) {
  servo(gpio, angle);
    // Task-4: 
    // Your code goes here (Use Lab 05 for reference)
    // Check the project document to understand the task
}

static uint8_t read_register(uint8_t reg_addr) {
    // Send START condition
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Send secondary device an address with write bit (0xEE = 0x77 << 1)
    TWDR = BMP180_WRITE;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Send register address to read from
    TWDR = reg_addr;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Send repeated START condition
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Send secondary device an address with read bit (0xEF = 0x77 << 1 | 1)
    TWDR = BMP180_READ;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Receive data byte, NACK (since we're only reading one byte)
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    uint8_t data = TWDR;

    // Send STOP condition
    TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);

    return data;
}

static int16_t read_temperature_adc()
{
uint8_t msb;
uint8_t lsb;

// Start temperature measurement
i2c_start();
i2c_write_address(BMP180_WRITE);      // Write mode
i2c_write(CTRL_MEAS_ADDR);            // Control register
i2c_write(CTRL_TEMP_CMND);            // Start temperature conversion
i2c_stop();

delay_us(4500);                       // Delay: 4.5ms for measurement, from Fig 3 in datasheet

// Read raw temperature value
i2c_start();                          // Start Condition
i2c_write_address(BMP180_WRITE);      // Write mode
i2c_write(ADC_MSB_ADDR);              // Target register: MSB register

delay_us(15);

i2c_start();
i2c_write_address(BMP180_READ);       // Read mode
msb = i2c_read_ack();                 // Read 0xF6 address, ack to continue reading
lsb = i2c_read_nack();                // Read 0xF7 address, nack to complete reading
i2c_stop();

/* Task 1.2 - Put together the msb and lsb into a 16-bit ut value to return it */
int16_t ut = (msb << 8) | lsb;

return (ut);
}

void setup() 
{
uart_init();     // Initialize UART for serial output
i2c_init();      // Initialize I2C for communication with BMP180

read_calibration( &calib_data );

//Setup Auto-break LEDs for Distance
	gpio_mode(GPIO_13, GPIO_OUTPUT); //RED
	gpio_mode(GPIO_12, GPIO_OUTPUT); //GREEN
	gpio_mode(GPIO_11, GPIO_OUTPUT); //BLUE

	gpio_mode(GPIO_8, GPIO_OUTPUT); //RED for temp
	gpio_mode(GPIO_9, GPIO_OUTPUT); //GREEN for temp
	gpio_mode(GPIO_10, GPIO_OUTPUT); //BLUE for temp
//Setup GPIO_6 for PWM output
	gpio_mode(GPIO_6, GPIO_OUTPUT);

	ser_printf("System Initialized");
	enable_timer_interrupt();
  set_cycles(25000); // Task 3.1 - Determine the amount of cycles for a 100ms interrupt timer
    //with div by 64 prescaler, ticks are 4us, so we can count to a max of 262144 us, we need to interrupt
    //every 100,000 us, so 100,000/ 4us per tick = 25,000 cycles

  enable_interrupt();
}

ISR(TIMER1_COMPA_vect) {
  intct++;
	char char_rcvd;
	char_rcvd = ser_read();
	auto_brake(char_rcvd);
}
/******************************************************************************
 *   Function: loop() - Main execution loop
 *      Pre condition: 
 *          setup() has been executed and system is initialized
 *      Post condition: 
 *          Performs a single iteration of the system's function
 *          Repeates indefinetely unless the board is reset or powered off
 *******************************************************************************/
void loop() {
// Task-4: 
	int gpio = GPIO_6;
// Setup simulated code for the angles from the lab sheet
	int angles[11] = {10, 25, 75, 45, 100, 40, 125, 15, 150, 50, 170};

  if (intct == 5) {
    intct = 0;
    steering(gpio, angles[aidx]);
      if (aidx + 1 == 11) aidx = 0;
      else aidx++;
  }

//  Task-4:
  int16_t ut = read_temperature_adc();                //raw ut adc value
	int32_t temperature = get_true_temperature( ut );   //tenths of degrees celsius
	int32_t temp_f = ((temperature * 9) / 5) + 320;     //convert to tenths of degrees farenheit
	int32_t tmp = temp_f / 10;
	if (tmp > 80) {
		gpio_write(GPIO_8, ON);
    gpio_write(GPIO_9, OFF);
    gpio_write(GPIO_10, OFF);
	} //turn on RED led
	else if (tmp > 70 && tmp <= 80) {
		gpio_write(GPIO_8, OFF);
    gpio_write(GPIO_9, ON);
    gpio_write(GPIO_10, OFF);
	} //turn on green LED
	else {
		gpio_write(GPIO_8, OFF);
    gpio_write(GPIO_9, OFF);
    gpio_write(GPIO_10, ON);
	}	//turn on blue LED
} //do steering and then do temp check

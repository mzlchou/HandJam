/* Header file for all EE14lib functions. */

#ifndef EE14LIB_H
#define EE14LIB_H

#include "stm32l432xx.h"
#include <stdbool.h>

// Pin names on the Nucleo silkscreen, copied from the Arduino Nano form factor
typedef enum {A0,A1,A2,A3,A4,A5,A6,A7,
	  D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13 } EE14Lib_Pin ;

typedef int EE14Lib_Err;

#define EE14Lib_Err_OK 0
#define EE14Lib_Err_INEXPLICABLE_FAILURE -1
#define EE14Lib_Err_NOT_IMPLEMENTED -2
#define EE14Lib_ERR_INVALID_CONFIG -3

// GPIO modes
#define INPUT 0b00
#define OUTPUT 0b01
// Normally shouldn't need to use ALTERNATE_FUNCTION directly; the peripheral modes will set this up
#define ALTERNATE_FUNCTION 0b10
#define ANALOG 0b11

// GPIO pullup modes
#define PULL_OFF 0b00
#define PULL_UP 0b01
#define PULL_DOWN 0b10
// Both on is an error

// GPIO output type modes
#define PUSH_PULL 0b0
#define OPEN_DRAIN 0b1

// GPIO output speed types
#define LOW_SPD 0b00
#define MED_SPD 0b01
#define HI_SPD  0b10
#define V_HI_SPD 0b11

EE14Lib_Err gpio_config_mode(EE14Lib_Pin pin, unsigned int mode);
EE14Lib_Err gpio_config_pullup(EE14Lib_Pin pin, unsigned int mode);
EE14Lib_Err gpio_config_otype(EE14Lib_Pin pin, unsigned int otype);
EE14Lib_Err gpio_config_ospeed(EE14Lib_Pin pin, unsigned int ospeed);
EE14Lib_Err gpio_config_alternate_function(EE14Lib_Pin pin, unsigned int function);
void gpio_write(EE14Lib_Pin pin, bool value);
bool gpio_read(EE14Lib_Pin pin);

EE14Lib_Err timer_config_pwm(TIM_TypeDef* const timer, const unsigned int freq_hz);
EE14Lib_Err timer_config_channel_pwm(TIM_TypeDef* const timer, const EE14Lib_Pin pin, const unsigned int duty);

void adc_init(void);
EE14Lib_Err adc_config_single(const EE14Lib_Pin pin);
unsigned int adc_read_single(void);

void i2c_init(I2C_TypeDef* i2c, EE14Lib_Pin scl, EE14Lib_Pin sda);
bool i2c_write(I2C_TypeDef* i2c, unsigned char device_address, unsigned char* data, unsigned char len);
bool i2c_read(I2C_TypeDef* i2c, unsigned char device_address, unsigned char* data, unsigned char len);

// Initialize the serial port
void host_serial_init();

// Very basic function: send a character string to the UART, one byte at a time.
// Spin wait after each byte until the UART is ready for the next byte.
void serial_write(USART_TypeDef *USARTx, const char *buffer, int len);

// Spin wait until we have a byte.
char serial_read(USART_TypeDef *USARTx);

#endif
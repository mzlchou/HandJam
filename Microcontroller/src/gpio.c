#include "ee14lib.h"

// Mapping of Nucleo pin number to GPIO port
static GPIO_TypeDef * g_GPIO_port[D13+1] = {
  GPIOA,GPIOA,GPIOA,GPIOA,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
  GPIOA,GPIOA,GPIOA,GPIOA,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
  GPIOA,GPIOA,GPIOA,GPIOB,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  GPIOB,GPIOB,GPIOB,GPIOC,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  GPIOC,GPIOA,GPIOA,GPIOB,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  GPIOB,GPIOB               // D12=PB4,D13=PB3.
};

// Mapping of Nucleo pin number to GPIO pin
// Using this plust g_GPIO_port[] above, we can translate a Nucleo pin name into
// the chip's actual GPIO port and pin number.
static uint8_t g_GPIO_pin[D13+1] = {
  0,1,3,4,    // A0=PA0,A1=PA1,A2=PA3,A3=PA4
  5,6,7,2,    // A4=PA5,A5=PA6,A6=PA7,A7=PA2
  10,9,12,0,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  7,6,1,14,   // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  15,8,11,5,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  4,3         // D12=PB4,D13=PB3.
};

// Enables a GPIO port (A, B, C, or H) by setting the appropriate bit in the RCC
// clock enable register.
//   gpio: Pointer to GPIO port to enable, one of GPIOA, GPIOB, GPIOC, GPIOH
static void gpio_enable_port (GPIO_TypeDef *gpio) {
    unsigned long field;
    if (gpio==GPIOA)      field=RCC_AHB2ENR_GPIOAEN;
    else if (gpio==GPIOB) field=RCC_AHB2ENR_GPIOBEN;
    else if (gpio==GPIOC) field=RCC_AHB2ENR_GPIOCEN;
    else           field=RCC_AHB2ENR_GPIOHEN;
    RCC->AHB2ENR |= field; // Turn on the GPIO clock
}


// Configure the direction for a given GPIO pin
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   direction: One of INPUT (0b00) or OUTPUT (0b01).  Other modes are invalid.
// Returns EE14Lib_ERR_INVALID_CONFIG for invalid direction value, otherwise
// returns EE14Lib_Err_OK.
EE14Lib_Err gpio_config_mode(EE14Lib_Pin pin, unsigned int mode)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    if(mode & ~0b11UL){ // Only bottom two bits are valid
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    // Enable the GPIO port in case it hasn't been already
    gpio_enable_port(port);

    port->MODER &= ~(0b11 << pin_offset*2); // Clear both mode bits
    port->MODER |=  (mode << pin_offset*2);

    return EE14Lib_Err_OK;
}
// Configure the direction for a given GPIO pin
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   direction: One of INPUT (0b00) or OUTPUT (0b01).  Other modes are invalid.
// Returns EE14Lib_ERR_INVALID_CONFIG for invalid direction value, otherwise
// returns EE14Lib_Err_OK.
EE14Lib_Err gpio_config_direction(EE14Lib_Pin pin, unsigned int direction)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    if(direction & ~0b1UL){ // Only 0b00 and 0b01 are valid directions
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    // Enable the GPIO port in case it hasn't been already
    gpio_enable_port(port);

    port->MODER &= ~(0b11 << pin_offset*2); // Clear both mode bits
    port->MODER |=  (direction << pin_offset*2);

    return EE14Lib_Err_OK;
}
// Configure the pullup mode for a given GPIO pin.  This is only meaningful when
// the pin is configured as an input, but it will set the registers regardless
// of mode.
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   mode: One of PULL_OFF (0b00), PULL_UP (0b01) or PULL_DOWN (0b10).  Other
//         modes are invalid.
// Returns EE14Lib_ERR_INVALID_CONFIG for invalid pullup mode value, otherwise
// returns EE14Lib_Err_OK.
EE14Lib_Err gpio_config_pullup(EE14Lib_Pin pin, unsigned int mode)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    if(mode & ~0b11UL){ // Only bottom two bits are are valid
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    port->PUPDR &= ~(0b11 << pin_offset*2); // Clear both mode bits
    port->PUPDR |=  (mode << pin_offset*2);

    return EE14Lib_Err_OK;
}

// Configure the output type of a GPIO pin, either push/pull or open-drain.
// A push-pull output is driven both high and low (connection to Vdd or GND),
// while an open-drain input is driven only to ground, and left floating
// otherwise.  A pullup (internal or external) is used to achieve the high value
// when the output is not driven low.
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   otype: Either PUSH_PULL (0b0) or OPEN_DRAIN (0b01)
// Returns EE14Lib_ERR_INVALID_CONFIG for invalid otype value, otherwise
// returns EE14Lib_Err_OK.
EE14Lib_Err gpio_config_otype(EE14Lib_Pin pin, unsigned int otype)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    if(otype & ~0b1UL){ // Only bottom bit is valid
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    port->OTYPER &= ~(0b1 << pin_offset); // Clear mode bit
    port->OTYPER |=  (otype << pin_offset);

    return EE14Lib_Err_OK;
}


// Configure the output speed of a GPIO pin.
EE14Lib_Err gpio_config_ospeed(EE14Lib_Pin pin, unsigned int ospeed)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    if(ospeed & ~0b11UL){ // Only bottom two bits are valid
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    port->OSPEEDR &= ~(0b11 << pin_offset*2); // Clear both speed bits
    port->OSPEEDR |=  (ospeed << pin_offset*2);

    return EE14Lib_Err_OK;
}

// Configure a GPIO pin for one of its "alternate functions".  See Tables 15 and
// 16 of the STM32L432KC datasheet for a complete listing of the alternate
// functions for each pin.
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   function: an integer 0-15 to select the alternate function
// Always returns EE14Lib_Err_OK; in the future this may return errors for
// invalid configurations.
EE14Lib_Err gpio_config_alternate_function(EE14Lib_Pin pin, unsigned int function)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    // Enable the GPIO port in case it hasn't been already
    gpio_enable_port(port);

    // Set the GPIO pin mode to "alternate function" (0b10)
    port->MODER &= ~(0b11 << pin_offset*2); // Clear both bits
    port->MODER |=  (0b10 << pin_offset*2); // 0b10 = alternate function mode

    // Set the AFR register
    unsigned int afr_offset = pin_offset * 4; // 4 bits per pin -> value from 0 to 60

    port->AFR[afr_offset >> 5] &= ~(0xF << (0x1F & afr_offset));
    port->AFR[afr_offset >> 5] |=  (function << (0x1F & afr_offset));

    return EE14Lib_Err_OK;
}

// Set the value of a single GPIO output pin.
//   pin: A Nucleo pin ID (D2, A4, etc.)
//   value: Boolean 0 or 1 to send to the pin
void gpio_write(EE14Lib_Pin pin, bool value)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];
    if(value){
      port->BSRR = 1 << pin_offset;
    }
    else{
      port->BRR = 1 << pin_offset; 
    }
}

// Read the value of a single GPIO pin.  This is only meaningful if the pin is
// configured as an input.
//   pin: A Nucleo pin ID (D2, A4, etc.)
// Returns a boolean, indicating the value (0/low or 1/high) of the pin
bool gpio_read(EE14Lib_Pin pin)
{
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];

    return (port->IDR >> pin_offset) & 1UL;
}


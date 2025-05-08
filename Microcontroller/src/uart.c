/* Helper functions for EE 14 Lab 1
 * www.ece.tufts.edu/ee/14
 * Steven Bell <sbell@ece.tufts.edu>, based almost entirely on work from Joel Grodstein
 * January 2025
 */
#include "stm32l432xx.h"
#include <stdbool.h>

static void USART_Init (USART_TypeDef *USARTx, bool tx_en, bool rx_en,int baud);
static void UART2_GPIO_Init(void);
static void USART_Delay(uint32_t us);

static void set_gpio_alt_func (GPIO_TypeDef *gpio,unsigned int pin,unsigned int func);

// The Nucleo 432 wires PA2 to the ST-Link's VCP_TX pin via AF7, and PA15 to
// VCP_RX via AF3.
// In this function, we set up those pins.
// The alternate-function designation presumably sets most on the GPIO pin's
// internals. However, we still set them here to high-speed, pullup-only,
// push-pull drive.
static void UART2_GPIO_Init(void) {
    set_gpio_alt_func (GPIOA,  2, 7);
    set_gpio_alt_func (GPIOA, 15, 3);

    // Set PA2 and PA15 to very-high-speed. This changes the output slew rate.
    GPIOA->OSPEEDR |=   0x3<<(2*2) | 0x3<<(2*15);

    // Both PA2 and PA15 are in pullup/down mode 01, which means pull-up only.
    // This is arguably not needed. During normal operation, we're doing push-
    // pull drive and so don't need a pullup or pulldown. Some people like a
    // pullup to stop the data line from bouncing during reset before any MCU
    // drives it -- but our pullup won't turn on until this code runs, anyway!
    GPIOA->PUPDR   &= ~((0x3<<(2*2)) | (0x3<<(2*15)));	// Clear bits
    GPIOA->PUPDR   |=   (0x1<<(2*2)) | (0x1<<(2*15));	// Set each to 01.

    // Both PA2 and PA15 are push-pull (which is the reset default, anyway).
    GPIOA->OTYPER  &= ~((0x3<<(2*2)) | (0x3<<(2*15)));	// Clear bits
}


// Set for 8 data bits, 1 start & 1 stop bit, 16x oversampling, 9600 baud.
// And by default, we also get no parity, no hardware flow control (USART_CR3),
// asynch mode (USART_CR2).
static void USART_Init (USART_TypeDef *USARTx, bool tx_en, bool rx_en,int baud){
    // Disable the USART.
    USARTx->CR1 &= ~USART_CR1_UE;  // Disable USART

    // The "M" field is two bits, M1 and M0. We're setting it to 00 (which
    // is the reset value anyway), to use 8-bit words and one start bit.
    USARTx->CR1 &= ~USART_CR1_M;

    // Configure stop bits to 1 stop bit (which is the default). Other
    // choices are .5, 1.5 and 2 stop bits.
    USARTx->CR2 &= ~USART_CR2_STOP;   

    // Set baudrate as desired. This is done by dividing down the APB1 clock.
    // E.g., 80MHz/9600 = 8333 = 0x208D.
    // (We're oversampling by 16; the calculation would be slightly
    // different if we were 8x mode).
    extern uint32_t SystemCoreClock;
    uint32_t val = SystemCoreClock / baud;
    USARTx->BRR  = val;

    // Configure oversampling mode: Oversampling by 16 (which is the
    // default). This means that our Rx runs at 16x the nominal baud rate.
    // If we're not enabling the Rx anyway, this step is moot (but harmless).
    USARTx->CR1 &= ~USART_CR1_OVER8;

    // Turn on transmitter and receiver enables. Note that the entire USART
    // is still disabled, though. Turning on the Rx enable kicks off the Rx
    // looking for a stop bit.
    if (tx_en)
	USARTx->CR1  |= USART_CR1_TE;
    if (rx_en)
	USARTx->CR1  |= USART_CR1_RE;
	
    // We originally turned off the USART -- now turn it back on.
    // Note that page 1202 says to turn this on *before* asserting TE and/or RE.
    USARTx->CR1  |= USART_CR1_UE; // USART enable                 
	
    // Verify that the USART is ready to transmit...
    if (tx_en)
	while ( (USARTx->ISR & USART_ISR_TEACK) == 0)
	    ;
    // ... and to receive.
    if (rx_en)
	while ( (USARTx->ISR & USART_ISR_REACK) == 0)
	    ;
}


void UART_write_byte (USART_TypeDef *USARTx, char data) {
    // spin-wait until the TXE (TX empty) bit is set
    while (!(USARTx->ISR & USART_ISR_TXE));

    // Writing USART data register automatically clears the TXE flag 	
    USARTx->TDR = data & 0xFF;

    // Wait 300us or so, to let the HW clear TXE.
    USART_Delay (300);
}

// Assume that each usec of delay is about 13 times around the NOP loop.
// That's probably about right at 80 MHz (maybe a bit too slow).
void USART_Delay(uint32_t us) {
    uint32_t time = 100*us/7;    
    while(--time);   
}


// Turn on the clock for a GPIO port.
static void gpio_enable_port (GPIO_TypeDef *gpio) {
    unsigned long field;
    if (gpio==GPIOA)      field=RCC_AHB2ENR_GPIOAEN;
    else if (gpio==GPIOB) field=RCC_AHB2ENR_GPIOBEN;
    else if (gpio==GPIOC) field=RCC_AHB2ENR_GPIOCEN;
    else 		  field=RCC_AHB2ENR_GPIOHEN;
    RCC->AHB2ENR |= field;			// Turn on the GPIO clock
}

// Set a given GPIO pin to be a particular alternate-function.
// Params:
//	gpio: which port; one of GPIOA, GPIOB, ... GPIOH.
//	pin:  0-15, for which GPIO pin in the port.
//	func: which of the 15 alternate functions to use.
void set_gpio_alt_func (GPIO_TypeDef *gpio, unsigned int pin, unsigned int func){
    gpio_enable_port (gpio);			// Turn on the GPIO-port clock.

    // Mode Register (MODER). Two bits of mode for each of the 16 pins/port.
    // And 10 -> alternate function.
    gpio->MODER &= ~(3UL << (2*pin));		// Clear the appropriate field.
    gpio->MODER |= 2UL << (2*pin);		// And set to binary 10.

    // AFRL sets the alternate function for pins 0-7; AFRH for pins 8-15.
    // Each register is just eight four-bit fields (one for each pin).
    // The .h file calls the two registers AFR[0] and AFR[1], but then names
    // the bits with the H and L suffixes!
    int idx = (pin>=8);
    int afr_pin = pin - 8*idx;
    gpio->AFR[idx] &= ~(0xFUL << (4*afr_pin));
    gpio->AFR[idx] |=  (func  << (4*afr_pin));

    // Output Speed Register (OSPEEDR). Two bits for each of the 16 pins/port.
    // And 00 -> low speed.
    gpio->OSPEEDR &= ~(3UL<<(2*pin));		// GPIO output speed=slow

    // Pull Up/Pull Down Register (PUPDR). Two bits for each of the 16
    // pins/port. And 00 -> no pullup or pulldown.
    gpio->PUPDR &= ~(3UL <<(2*pin));		// No PUP or PDN
}


void host_serial_init() {
    int baud=9600;

    // Enable USART 2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;  

    // Select SYSCLK as the USART2 clock source. The reset default is PCLK1;
    // we usually set both SYSCLK and PCLK1 to 80MHz anyway.
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
    RCC->CCIPR |=  RCC_CCIPR_USART2SEL_0;

    // Connect the I/O pins to the serial peripheral
    UART2_GPIO_Init();

    USART_Init (USART2, 1, 1, baud);	// Enable both Tx and Rx sides.

}

// Very basic function: send a character string to the UART, one byte at a time.
// Spin wait after each byte until the UART is ready for the next byte.
void serial_write (USART_TypeDef *USARTx, const char *buffer, int len) {
    // The main flag we use is Tx Empty (TXE). The HW sets it when the
    // transmit data register (TDR) is ready for more data. TXE is then
    // cleared when we write new data in (by a write to the USART_DR reg).
    // When the HW transfers the TDR into the shift register, it sets TXE=1.
    for (unsigned int i = 0; i < len; i++) {
	    UART_write_byte (USARTx, buffer[i]);
    }

    // RM0394 page 1203 says that you must wait for ISR.TC=1 before you shut
    // off the USART. We never shut off the USART... but we'll wait anyway.
    while (!(USARTx->ISR & USART_ISR_TC));
    USARTx->ISR &= ~USART_ISR_TC;
}

char serial_read (USART_TypeDef *USARTx) {
    // The SR_RXNE (Read data register not empty) bit is set by hardware.
    // We spin wait until that bit is set
    while (!(USARTx->ISR & USART_ISR_RXNE))
	;

    // Reading USART_DR automatically clears the RXNE flag 
    return ((char)(USARTx->RDR & 0xFF));
}

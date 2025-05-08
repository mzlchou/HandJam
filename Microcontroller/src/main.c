#include "stm32l432xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "ee14lib.h"

#define AUDIO_PIN A1       // PA1
#define SD_PIN    A0       // PA0 
#define TICK_HZ   1000000

//serial port
int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}
//7 seg
int digits[10]={0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011, 0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1110011};

void display(int number){
    int a = digits[number] & 0b1000000;
    int b = digits[number] & 0b0100000;
    int c = digits[number] & 0b0010000;
    int d = digits[number] & 0b0001000;
    int e = digits[number] & 0b0000100;
    int f = digits[number] & 0b0000010;
    int g = digits[number] & 0b0000001;
    gpio_write(D3, a);
    gpio_write(D9, b);
    gpio_write(D4, c);
    gpio_write(D6, d);
    gpio_write(D5, e);
    gpio_write(D0, f);
    gpio_write(D1, g);
}
//Explain this
static void audio_pwm_init(void)
{
    gpio_config_alternate_function(AUDIO_PIN, 1);          /* AF1 for TIM2_CH2 */
    gpio_config_mode(AUDIO_PIN, ALTERNATE_FUNCTION);
    gpio_config_ospeed(AUDIO_PIN, HI_SPD);

    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;                  /* Enable TIM2 clock */
    TIM2->PSC = (SystemCoreClock / TICK_HZ) - 1;           /* Set prescaler */
    TIM2->CCER = TIM_CCER_CC2E;                            /* Enable CH2 output */
    TIM2->CR1 = TIM_CR1_CEN;                               /* Enable TIM2 */
}

//JONAH WHAT IS THIS
static void play(float f_hz, uint16_t ms)
{
    if (f_hz <= 0) return;

    uint32_t arr = (uint32_t)(TICK_HZ / f_hz - 1);
    TIM2->ARR  = arr;
    TIM2->CCR2 = arr / 2;  // 50% duty cycle

    // Clear and configure CCMR1 for Channel 2 PWM Mode 1 with preload enabled
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM2->CCMR1 |= (0x6 << 12) | TIM_CCMR1_OC2PE;

    // Force update to load ARR and CCR2
    TIM2->EGR = TIM_EGR_UG;

    // crude delay loop
    for (uint32_t i = 0; i < ms * (SystemCoreClock / 8000UL); i++) __NOP();
}
/////////////////////

//play note
// void play_note(float fz){
//     play(fz, 100); 
// }

//SYSTICK STUFF
volatile int counter = 0;
void  SysTick_Handler(void) {
    counter++;
}
void delay_ms(volatile int input_ms){
    volatile int intermediate = counter;
    while((counter-intermediate)<input_ms){}
}
////////////////////////////


//initializing sys_tick
void SysTick_initialize(void) {
    // TODO: figure out what each line of code in this function does
    SysTick->CTRL = 0; //initializing/clearing any possible configurations from before
    SysTick->LOAD = 3999; //divide by load+1 4MHZ/1khz


    // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
    // largest supported value (aka lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0; //clear current SysTick value
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //using processor clk
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //enabling interupts
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable timer
}

int main() {
    //init
    SysTick_initialize();
    host_serial_init();
    gpio_config_direction(D1, 0b01);
    gpio_config_direction(D0, 0b01);
    gpio_config_direction(D5, 0b01);
    gpio_config_direction(D6, 0b01);
    gpio_config_direction(D9, 0b01);
    gpio_config_direction(D3, 0b01);
    gpio_config_direction(D4, 0b01);
    //char rx_buffer[16]; //character array to store incoming uart data
    

#ifdef SD_PIN
    gpio_config_mode(SD_PIN, OUTPUT); //un mute amp
    gpio_write(SD_PIN, true);
#endif

    audio_pwm_init();

    const float scale[] = {
        261.63f,  //C4
        293.66f,  //D4
        329.63f,  //E4
        349.23f,  //F4
        392.00f,  //G4
        440.00f,  //A4
        493.88f,  //B4
        523.25f   //C5
    };

    volatile int x_val = 0;
    volatile int prev_y_val = 0;
    volatile int y_val = 0;

    while (1) {
        // while (USART2->ISR & USART_ISR_RXNE) {
        //     volatile char dump = USART2->RDR; //clear the buffer
        // }    
        // int value= 1;
        //printf("%d\n", value); //sending the 1

        while (!(USART2->ISR & USART_ISR_RXNE));
        char c = USART2->RDR;
        
        while (!(USART2->ISR & USART_ISR_RXNE));  //wait for and discard newline char
        volatile char discard = USART2->RDR;
        
        int x = c - '0';  //ASCII to int | x stores model prediction
        
        
        adc_config_single(A2);
        x_val = (adc_read_single());
        // printf("x axis %d\n", x_val);
        adc_config_single(A3);
        y_val = (adc_read_single());
        // printf("y axis: %d\n", y_val);
        display(x); //7seg display
        if (x == 8){
            x=7;
        }

        if (x==9){
            x=4;
        }

        // int fz_change = 0;
        // if(x_val > 3000){
        //     int u = x_val - 2900; // 1000 and 0
        //     fz_change = (u / 40);
        // }
        // else if (x_val < 2800){
        //     fz_change = (x_val / -93);
        // }
        if(y_val <2500){
         play(scale[x], 50);
        }
        else{
            TIM2->CCR2 = 0; //turn pwm off duty cycle 0
        }

        // Pause
                
        // if (x==4){
        //     gpio_write(A0, 1);
        // }
        // else{
        //     gpio_write(A0, 0);
        // }
        // printf("STM32 got: %d\n", x);
    }
}

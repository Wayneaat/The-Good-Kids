#include "stm32f0xx.h"
#include <string.h> // for memset()
#include <stdio.h> // for printf()
#include "ff.h"
#include "fifo.h"
#include "tty.h"
#include <diskio.h>
#include <math.h>
#include <stdlib.h>

char line[21];

void internal_clock(void);
void display_float(float);
void control(void);

int THRESHOLD = 630000; //total timer
int count = 0;
int LED = 1;
float adc_in = 0;

//============================================================================
// setup_adc()    (Autotest #1)
// Configure the ADC peripheral and analog input pins.
// Parameters: none
//============================================================================
void setup_adc(void)
{

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= 0Xf00;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;
	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR && ADC_ISR_ADRDY));

}

//============================================================================
// start_adc_channel()    (Autotest #2)
// Select an ADC channel, and initiate an A-to-D conversion.
// Parameters: n: channel number
//============================================================================
void start_adc_channel(int n)
{
	ADC1->CHSELR = 0;
	ADC1->CHSELR = 1<<n;
	while(!(ADC1->ISR && ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;

}

//============================================================================
// read_adc()    (Autotest #3)
// Wait for A-to-D conversion to complete, and return the result.
// Parameters: none
// Return value: converted result
//============================================================================
float read_adc(void)
{
	while(!(ADC1->ISR & ADC_ISR_EOC));
	//sprintf(line, "PA5: %d", ADC1->DR * 3 / 4095.0);
	// adc_in = ADC1->DR * 3.0 / 4095.0;
    return ADC1->DR;
}

//============================================================================
// setup_dac()    (Autotest #4)
// Configure the DAC peripheral and analog output pin.
// Parameters: none
//============================================================================
void setup_dac(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= 3<<(2*4);
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_TSEL1;
	DAC->CR |= DAC_CR_TEN1;
	DAC->CR |= DAC_CR_EN1;

}

void close_dac(){
	DAC->CR &= ~DAC_CR_EN1;
}
void enable_dac(){
	DAC->CR |= DAC_CR_EN1;
}
//============================================================================
// write_dac()    (Autotest #5)
// Write a sample to the right-aligned 12-bit DHR, and trigger conversion.
// Parameters: sample: value to write to the DHR
//============================================================================
void write_dac(int sample)
{
	DAC->DHR12R1 = sample;
	DAC->SWTRIGR = DAC_SWTRIGR_SWTRIG1;
}


//============================================================================
// Parameters for the wavetable size and expected DAC rate.
//============================================================================
#define N 1000
#define RATE 20000
short int wavetable[N];

//============================================================================
// init_wavetable()    (Autotest #6)
// Write the pattern for one complete cycle of a sine wave into the
// wavetable[] array.
// Parameters: none




//============================================================================
void init_wavetable(void)
{
	for(int i=0; i < N; i++)
		           wavetable[i] = 32767 * sin(2 * M_PI * i / N);

}

//============================================================================
// Global variables used for four-channel synthesis.
//============================================================================
int volume = 2048;
int stepa = 0;
int stepb = 0;
int stepc = 0;
int stepd = 0;
int offseta = 0;
int offsetb = 0;
int offsetc = 0;
int offsetd = 0;

//============================================================================
// set_freq_n()    (Autotest #7)
// Set the four step and four offset variables based on the frequency.
// Parameters: f: The floating-point frequency desired.
//============================================================================

void set_freq_a(float f)
{
	stepa = f*N/RATE*(1<<16);
	if(f==0){
		offseta =0;
		stepa = 0;
	}
}

void set_freq_b(float f)
{
	stepb = f*N/RATE*(1<<16);
	if(f==0){
		offsetb =0;
		stepb = 0;
	}
}
void set_freq_c(float f)
{
	stepc = f*N/RATE*(1<<16);
	if(f==0){
		offsetc =0;
		stepc = 0;
	}
}
void set_freq_d(float f)
{
	stepd = f*N/RATE*(1<<16);
	if(f==0){
		offsetd =0;
		stepd = 0;
	}
}

//============================================================================
// Timer 6 ISR    (Autotest #8)
// The ISR for Timer 6 which computes the DAC samples.
// Parameters: none
// (Write the entire subroutine below.)
//============================================================================

void TIM6_DAC_IRQHandler(){

//		count++; //set LED light timer
//		if(count < 50000 ) LED = 4;
//		else if(count < 75000 ) LED = 2;
//		else LED = 4;
//		if(count == THRESHOLD)count = 0;
		GPIOC->ODR = LED; //pc0, 1, 2; odr = voltage
		// LED=1; //TEST WAVE


//		if(LED != 1) close_dac();
//		else enable_dac();
//        // new code //
//        if(LED == 1) enable_dac();
//        else close_dac();


		TIM6 ->SR &= ~TIM_SR_UIF;
		DAC->SWTRIGR = DAC_SWTRIGR_SWTRIG1;

		offseta += stepa;
		offsetb += stepb;
		offsetc += stepc;
		offsetd += stepd;

		if (offseta >= (N<<16)){
			offseta -= N<<16;
		}
		if (offsetb >= (N<<16)){
					offsetb -= N<<16;
		}
		if (offsetc >= (N<<16)){
					offsetc -= N<<16;
		}
		if (offsetd >= (N<<16)){
					offsetd -= N<<16;
		}
		float sum=0;
		sum = wavetable[offseta>>16]+wavetable[offsetb>>16]
	          +wavetable[offsetc>>16]+wavetable[offsetd>>16];
		sum = sum/12; //change amplitude /12
		sum +=2048;
		if (sum<0) sum = 0;
		if (sum > 4095) sum =4095;
		DAC->DHR12R1 = sum;

}

//============================================================================
// setup_tim6()    (Autotest #9)
// Configure Timer 6 to raise an interrupt RATE times per second.
// Parameters: none
//============================================================================
void setup_tim6(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 239;
	TIM6->ARR = 9;
	TIM6->CR1 &= ~TIM_CR1_DIR;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC->ISER[0] |= 1<<TIM6_DAC_IRQn;
}



void setup_usart2(void)
{
 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


 GPIOA->MODER |= 2<<(2*2);
 GPIOA->MODER |= 2<<(2*3) ;

 GPIOA->AFR[0] |= 1<<(4*2) ;
 GPIOA->AFR[0] |= 1<<(4*3) ;
 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

 USART2->CR1 &= ~USART_CR1_UE;
 USART2->BRR = 480000 / 384;
 USART2->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
 while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
}

void USART2_IRQHandler (void)
{

	if (USART2->ISR & USART_ISR_ORE)
		USART2->ICR |= USART_ICR_ORECF;
	char ch = USART2->RDR;
	if(fifo_full(&input_fifo))
		return;
	insert_echo_char(ch);

}

void enable_tty_interrupt (void)
{
	USART2->CR1 |= USART_CR1_RXNEIE;
	NVIC->ISER[0] = (1<<28);
}

int interrupt_getchar()
{
     while(fifo_newline(&input_fifo)==0)
       asm volatile ("wfi"); // wait for an interrupt

     char ch = fifo_remove(&input_fifo);
     return ch;
}

int better_putchar(int x)
{
 while ((USART2->ISR & USART_ISR_TXE) != USART_ISR_TXE);
 if (x == '\n')
 {
  while ((USART2->ISR & USART_ISR_TXE) != USART_ISR_TXE);
  USART2->TDR = '\r';
  while ((USART2->ISR & USART_ISR_TXE) != USART_ISR_TXE);
  USART2->TDR = '\n';
 }
 else
  USART2->TDR = x;

 return x;
}

int __io_putchar(int ch)
{
 return better_putchar(ch);
}

int __io_getchar(void)
{
 return interrupt_getchar();
}

void mywait(int x) {
asm volatile(" mov r0, %0\n"

"again:\n"
" nop\n"
" nop\n"
" nop\n"
" nop\n"
" sub r0, #1\n"
" bne again\n"
: : "r"(x) : "r0", "cc");
}



int main(void)
{
    //internal_clock(); // Use the internal oscillator if you need it
    //autotest(); // test all of the subroutines you wrote
        init_wavetable();
    setup_dac();
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0X7;
    GPIOC->MODER |= 0X55;
    GPIOC->ODR &= ~0X7;
//    int x = 1;
//    GPIOC->ODR |= x; //change lights
    setup_adc();
    setup_tim6();

    // try frequency
//    set_freq_a(500); //freq 1k test
//    set_freq_a(261.626); // Middle 'C'
    set_freq_b(330); // The 'E' above middle 'C' 329.628
    //control();
//    if (LED != 1) close_dac();
//    else setup_dac();

    setbuf(stdin, 0);
    setbuf(stdout, 0);
    setbuf(stderr, 0);
    setup_usart2();
    enable_tty_interrupt(); // slave code
    int low;
    int hi;
    int audio_counter = 0;
    		//audio_stop_counter = 0;
    while(1) {
//        for(int out=0; out<4096; out++) {
//            if ((TIM6->CR1 & TIM_CR1_CEN) == 0)
//                write_dac(out);

            start_adc_channel(5);
            float sample = read_adc();
            int x = (int)((sample * 300 / 4095) + 0.5) + 33; // x = real voltage value

            /*// master start
            mywait(500000);
            better_putchar(x);
            int value = x - 33;
			*/// master end

            //slave start
            mywait(300000);
            char y = __io_getchar();
            //printf("==%c", y);
            int a = ((int)y) - 33;

            a = a/2;
            if (a >= 50) {
            	a = a - 7.5;
            }

            //else a = a+1;

            //int value = a;

            if (x>=a) {
            	low = a;
            	hi = x -33;
            }
            else if (x < a) {
            	low = x -33;
            	hi = a;
            }
            int value = ( low  + hi * 4 ) / 5; // average value
            //int value = x;
			// slave end


			//LED = 1;
            // 50-70 9 14 34 39
            // 65-80 15 19 45 50
            // 4/7value 8 11 46 51
            //main 13 18 38 43
            //apart 18 23 75 80

            //apart 40 120
            //main 23 60
            // 15 20 57 62
            //4/15 main 12 15 45 53
            //4/15 bluetooth 23 29 140 145
            if(value >= 53) LED = 1;
            else if (value >= 10  && value <= 45) LED = 2;
            else if (value < 8) LED = 4;

            // if LED=1(red),  turn on DAC speaker.
            if(LED == 1) {
            	enable_dac();

            	audio_counter++;
            	if(audio_counter >= 15){
//            		audio_counter = 0;
            		close_dac();
            	}
            	if(audio_counter >= 20) audio_counter = 0;

            }
            else close_dac();

            //float level = 2.95 * sample / 4095;
            //display_float(level);
        }

    return 0;
}

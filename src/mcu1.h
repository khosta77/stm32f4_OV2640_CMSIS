#ifndef MCU1_H_
#define MCU1_H_

#include "../system/include/cmsis/stm32f4xx.h"

#define CFGR_MCO1_RESET_MASK      ((uint32_t)0xF89FFFFF)

#define RCC_MCO1Source_HSI               ((uint32_t)0x00000000)
#define RCC_MCO1Source_LSE               ((uint32_t)0x00200000)  // сигнала нет, на 1
#define RCC_MCO1Source_HSE               ((uint32_t)0x00400000)  // сигнала нет, на 1
#define RCC_MCO1Source_PLLCLK            ((uint32_t)0x00600000)  // сигнала нет, на 1

#define RCC_MCO1Div_1                    ((uint32_t)0x00000000)
#define RCC_MCO1Div_2                    ((uint32_t)0x04000000)
#define RCC_MCO1Div_3                    ((uint32_t)0x05000000)
#define RCC_MCO1Div_4                    ((uint32_t)0x06000000)
#define RCC_MCO1Div_5                    ((uint32_t)0x07000000)

#define RCC_MCO1Source RCC_MCO1Source_HSI
#define RCC_MCO1Div    RCC_MCO1Div_1

void MCO1_init() {
    //enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    //PA8 -> MCO(AF) -> PLLCLK
    GPIOA->MODER |= GPIO_MODER_MODER8_1;      // AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;       // PP
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;      // PU
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; // High speed  // 100MHz

    //AF0 -> MCO
    GPIOA->AFR[1] |= (0x0 << 0);

    RCC->CFGR &= CFGR_MCO1_RESET_MASK;
    RCC->CFGR |= RCC_MCO1Source | RCC_MCO1Div;
//    RCC->CFGR &= ~RCC_CFGR_MCO1; //HSI
}

void PWM_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (0x2 << (2 * 12));
    GPIOD->AFR[1] |= (0x2 << 16); 
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 2;
    TIM4->ARR = 5;
    TIM4->CCMR1 |= 0x60;
    TIM4->CCR1 = 2;
    TIM4->CCER |= 0x1;
    
    TIM4->DIER |= TIM_DIER_UIE;    
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 2);
    
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(void) {
	TIM4->SR &= ~TIM_SR_UIF;
	//TIM4->CR1 &= ~TIM_CR1_CEN;   
    //TIM4->CR1 |= TIM_CR1_CEN;
}

#endif  // MCU1_H_

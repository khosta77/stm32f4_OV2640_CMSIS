#ifndef MCU1_H_
#define MCU1_H_

#include "../system/include/cmsis/stm32f4xx.h"

#define CFGR_MCO1_RESET_MASK      ((uint32_t)0xF89FFFFF)

#define RCC_MCO1Source_HSI               ((uint32_t)0x00000000)
#define RCC_MCO1Source_LSE               ((uint32_t)0x00200000)
#define RCC_MCO1Source_HSE               ((uint32_t)0x00400000)
#define RCC_MCO1Source_PLLCLK            ((uint32_t)0x00600000)

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

    uint32_t tmpreg = 0;
    tmpreg = RCC->CFGR;
    tmpreg &= CFGR_MCO1_RESET_MASK;
    tmpreg |= RCC_MCO1Source | RCC_MCO1Div;

    RCC->CFGR = tmpreg;  
//    RCC->CFGR &= ~RCC_CFGR_MCO1; //HSI
}



#endif  // MCU1_H_

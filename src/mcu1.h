#ifndef MCU1_H_
#define MCU1_H_

#include "../system/include/cmsis/stm32f4xx.h"


void MCO1_init() {
#if 1
    //enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    //PA8 -> MCO(AF) -> PLLCLK
    GPIOA->MODER |= GPIO_MODER_MODER8_1;      // AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;       // PP
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;      // PU
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; // High speed

    //AF0 -> MCO
    GPIOA->AFR[1] |= (GPIO_AF_MCO << 0);
    RCC->CFGR &= ~RCC_CFGR_MCO1; //HSI
#else
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC_ClockSecuritySystemCmd(ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//PA8 - XCLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
#endif
}



#endif  // MCU1_H_

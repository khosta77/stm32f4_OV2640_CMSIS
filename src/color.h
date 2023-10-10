#ifndef COLOR_H_
#define COLOR_H_

#include "../system/include/cmsis/stm32f4xx.h"

#define PWM_MODE 1

#if PWM_MODE
#define G_ON() { GPIOD->ODR |= GPIO_ODR_OD12; }
#define G_OFF() { GPIOD->ODR &= ~GPIO_ODR_OD12; }
#endif

#define O_ON() { GPIOD->ODR |= GPIO_ODR_OD13; }
#define O_OFF() { GPIOD->ODR &= ~GPIO_ODR_OD13; }

#define R_ON() { GPIOD->ODR |= GPIO_ODR_OD14; }
#define R_OFF() { GPIOD->ODR &= ~GPIO_ODR_OD14; }

#define B_ON() { GPIOD->ODR |= GPIO_ODR_OD15; }
#define B_OFF() { GPIOD->ODR &= ~GPIO_ODR_OD15; }

void my_GPIO_init();  // Для вывода цветов

void my_GPIO_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
#if PWM_MODE
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
#else
    GPIOD->MODER |= (GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOD->ODR &= ~(GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
#endif
}

#endif  // COLOR_H_

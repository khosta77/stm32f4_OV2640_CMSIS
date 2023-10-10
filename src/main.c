#include "../system/include/cmsis/stm32f4xx.h"

#include "./color.h"
#include "./mcu1.h"
#include "./dcmi.h"
#include "./OV7725.h"

#define DELAY(time_) {for (uint32_t t = 0; t < time_; t++);}

uint32_t img[(IMG_ROWS * IMG_COLS)] = {0};

//#include "../system/include/stm32f4-hal/stm32f4xx_hal.h"
//#include "../system/include/stm32f4/stm32f4xx_rcc.h"
//#include "../system/include/stm32f4/stm32f4xx_gpio.h"
//#include "../system/include/stm32f4/stm32f4xx_i2c.h"
//#include "../system/include/stm32f4/stm32f4xx_dma.h"
//#include "../system/include/stm32f4/stm32f4xx_dcmi.h"
//#include "../system/include/stm32f4/misc.h"

void init();
void screen();

//===========================================================================================================
/*                                  main                                                                   */
//===========================================================================================================
int main(void) {
    init();
	while(1) {
        screen();
	}
}

void init() {
    //SysTick_Config(SystemCoreClock);
    my_GPIO_init();
    MCO1_init();
    //PWM_init();
    DCMI_init(); 
    OV7725_init();
    DMA2_Stream1->CR |= DMA_SxCR_EN;
}

void screen() {
    switch (image_state) {
        case 0x00: {
            O_ON();
            DELAY(100000);
            O_OFF();
            break;
        }
        case 0x01: {
            image_state = 0x00;
            DMA2_Stream1->CR |= DMA_SxCR_EN;
            B_ON();
            O_ON();
            R_ON();
            DELAY(100000);
            R_OFF();
            O_OFF();
            B_OFF();
            break;
        }
        case 0x02: {
            image_state = 0x00;
            DMA2_Stream1->CR |= DMA_SxCR_EN;
            R_ON();
            DELAY(10000);
            R_OFF();
            break;
        }
        default: {
            B_ON();
            DELAY(10000);
            B_OFF();
        }
    };
}

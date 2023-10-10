#include "../system/include/cmsis/stm32f4xx.h"

#include "./color.h"
#include "./mcu1.h"
#include "./dcmi.h"
#include "./OV7725.h"

#define DELAY(time_) {for (uint32_t t = 0; t < time_; t++);}

uint32_t img[(IMG_ROWS * IMG_COLS)] = {0};

//#include "../system/include/stm32f4-hal/stm32f4xx_hal.h"
//#include "../system/include/stm32f4/stm32f4xx_rcc.h"
#include "../system/include/stm32f4/stm32f4xx_gpio.h"
#include "../system/include/stm32f4/stm32f4xx_i2c.h"
//#include "../system/include/stm32f4/stm32f4xx_dma.h"
//#include "../system/include/stm32f4/stm32f4xx_dcmi.h"
//#include "../system/include/stm32f4/misc.h"

void init();
void screen();

//===========================================================================================================
/*                                  main                                                                   */
//===========================================================================================================

void new_I2C() {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_I2C2);

	// I2C config
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

}

int main(void) {
    my_GPIO_init();
    MCO1_init();

    uint8_t mmm;

	InitI2C0();

    uint8_t regnum_0 = OVxxxx_ReadReg(OV7725_DEVICE_READ_ADDRESS, 0x1C);
    uint8_t regnum_1 = OVxxxx_ReadReg(OV7725_DEVICE_READ_ADDRESS, 0x1D);

    if (regnum_0 == regnum_1)
        GPIOD->ODR |= GPIO_ODR_OD12;
    
    if (regnum_0 == 0x00)
        GPIOD->ODR |= GPIO_ODR_OD13;

    if (regnum_1 == 0x00)
        GPIOD->ODR |= GPIO_ODR_OD14;
    while (1) {;}
	//OVxxxx_WriteReg(OV7725_DEVICE_WRITE_ADDRESS, 0xff, 0x01);
   	//mmm = 0x80;
	//if (0 == OVxxxx_WriteReg(OV7725_DEVICE_WRITE_ADDRESS, 0x12, mmm)) {
	//	return 0;
	//}
	//delay_ms(10);

	//OV7725_set_register();
	//return 1;

    //init();
	//while(1) {
      //  screen();
	//}
}

void init() {
    //SCCB_init();
    MCO1_init();
    //SysTick_Config(SystemCoreClock);
    //my_GPIO_init();
    //MCO1_init();
    //PWM_init();
    //DCMI_init(); 
    //OV7725_init();
    //DMA2_Stream1->CR |= DMA_SxCR_EN;
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

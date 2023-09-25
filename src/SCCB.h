#ifndef SERIAL_CAMERA_CONTROL_BUS_H_
#define SERIAL_CAMERA_CONTROL_BUS_H_

#include "../system/include/cmsis/stm32f4xx.h"

volatile uint32_t timestamp = 0;

void SysTick_Handler(void) {
   //сюда попадаем каждую 1 миллисекунду
   timestamp++;
}

void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#define RCC_AHB1ENR_SCCBEN() { \
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN); \
}

#define GPIO_SCCB_SIO_D_INIT() { \
    GPIOB->MODER |= GPIO_MODER_MODER10_0; \
/*    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; \ */ \
    GPIOB->ODR &= ~GPIO_ODR_OD10; \
}

#define SIO_D_ON() { \
    GPIOB->ODR |= GPIO_ODR_OD10; \
}

#define SIO_D_OFF() { \
    GPIOB->ODR &= ~GPIO_ODR_OD10; \
}

#define SIO_D_IN_MODE() { \
    GPIOB->MODER &= ~(GPIO_MODER_MODER10_1 | GPIO_MODER_MODER10_0); \
}

#define SIO_D_OUT_MODE() { \
    GPIOB->MODER &= ~(GPIO_MODER_MODER10_1 | GPIO_MODER_MODER10_0); \
    GPIOB->MODER |= GPIO_MODER_MODER10_0;
}

#define GPIO_SCCB_SIO_C_INIT() { \
    GPIOB->MODER |= GPIO_MODER_MODER11_0; \
/*    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11; \ */ \
    GPIOB->ODR &= ~GPIO_ODR_OD11; \
}

#define SIO_C_ON() { \
    GPIOB->ODR |= GPIO_ODR_OD11; \
}

#define SIO_C_OFF() { \
    GPIOB->ODR &= ~GPIO_ODR_OD11; \
}

#define READ_SIO_D ((GPIOB->IDR >> 11) & 0x00000001)

void InitI2C0();
void StartI2C0();
void StopI2C0();
void NoAck0();
void Ack0();
uint8_t TestAck0();
uint8_t I2CWrite0(uint8_t DData);
uint8_t I2CRead0();

void InitI2C0() {
    RCC_AHB1ENR_SCCBEN();
    GPIO_SCCB_SIO_D_INIT();
    GPIO_SCCB_SIO_C_INIT();
}

void StartI2C0() {
    SIO_D_ON();
    delay_us(100);

    SIO_C_ON();
    delay_us(100);
 
    SIO_D_OFF();
    delay_us(100);

    SIO_C_OFF();
    delay_us(100);
}

void StopI2C0() {
    SIO_D_OFF();
    delay_us(100);
 
    SIO_C_OFF();
    delay_us(100);  

    SIO_D_ON();
    delay_us(100);
}

void NoAck0() {
    SIO_D_ON();
	delay_us(100);
	
    SIO_C_ON();
	delay_us(100);
	
    SIO_C_OFF();
	delay_us(100);
	
    SIO_D_OFF();
	delay_us(100);
}

void Ack0() {
    SIO_D_OUT_MODE();

    SIO_D_ON();
    delay_us(100); 

    SIO_C_ON();
    delay_us(100); 

    SIO_C_OFF();
    delay_us(100); 

    SIO_D_ON();
}

uint8_t TestAck0() {
	uint8_t ack;

    SIO_C_ON();
	delay_us(100);

    SIO_D_IN_MODE();
	delay_us(100);
	
    ack = READ_SIO_D;  // SDA_STATE0;  // Считываем сигнал
	delay_us(100);
	
    SIO_C_OFF();
	delay_us(100);
	return ack;
}

uint8_t I2CWrite0(uint8_t DData) {
	uint8_t j, tem;

	for (j = 0; j < 8; j++) {
		if ((DData << j) & 0x80) {
            SIO_D_ON();
		} else{
            SIO_D_OFF();
		}
		delay_us(100);
		
        SIO_C_ON();
		delay_us(100);
		
        SIO_C_OFF();
		delay_us(100);
	}
	delay_us(100);
	
    SIO_D_IN_MODE();
	delay_us(100);

    SIO_C_ON();
	delay_us(1000);

    if (READ_SIO_D /* SDA_STATE0 */ == 1) {
		tem = 0;  
	} else {
		tem = 1;   
	}
    SIO_C_OFF();
	delay_us(100);

    SIO_D_OUT_MODE();
	return tem;  
}

uint8_t I2CRead0() {
	uint8_t read, j;
	read = 0x00;
	
    SIO_D_IN_MODE();
	delay_us(100);
	for (j = 8; j > 0; j--) {
		delay_us(100);

        SIO_C_ON();
		delay_us(100);
		
        read = read << 1;
		if (READ_SIO_D /* SDA_STATE0 */ == 1) {
			read = read + 1;
		}
        SIO_C_OFF();
		delay_us(100);
	}

	return read;
}

void delay_ms(uint16_t nms) {
    uint16_t delay = (timestamp + nms);
    while (delay < timestamp);
}   

void delay_us(uint32_t nus) {
    uint32_t delay = (timestamp + nus);
    while (delay < timestamp);
}

#endif  // SERIAL_CAMERA_CONTROL_BUS_H_

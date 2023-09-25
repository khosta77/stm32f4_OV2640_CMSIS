#ifndef OV2640_H_
#define OV2640_H_

#include "OVxxxx_foo.h"
#include "OV2640_register.h"

#if 0
void OV2640_HW_Init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11;
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;  // SCL | SDA
    GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
    GPIOB->AFR[1] |= ((0x4 << 8) | (0x4 << 12));//GPIO_AFRH_AFRH10_2 | GPIO_AFRH_AFRH11_2;
#if 0
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    I2C2->CR2 = 0x2A;
    I2C2->CCR = 0xD2;
    I2C2->TRISE = 0x2B;
    I2C2->CR1 |= I2C_CR1_PE;
#else
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    I2C_InitTypeDef  I2C_InitStruct;

    /* Configure I2C2 */
    /* I2C DeInit */
    I2C_DeInit(I2C2);

    /* Set the I2C structure parameters */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 30000;

    /* Initialize the I2C peripheral w/ selected parameters */
    I2C_Init(I2C2, &I2C_InitStruct);

    /* Enable the I2C peripheral */
    I2C_Cmd(I2C2, ENABLE);
#endif
}

#if 0
#else
uint8_t OV2640_WriteReg(uint16_t Addr, uint8_t Data) {
    uint32_t timeout = DCMI_TIMEOUT_MAX;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    /* Test on I2C2 EV5 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C2, OV2640_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

    /* Test on I2C2 EV6 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send I2C2 location address LSB */
    I2C_SendData(I2C2, (uint8_t)(Addr));

    /* Test on I2C2 EV8 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send Data */
    I2C_SendData(I2C2, Data);

    /* Test on I2C2 EV8 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send I2C2 STOP Condition */
    I2C_GenerateSTOP(I2C2, ENABLE);

    /* If operation is OK, return 0 */
    return 0;
}
#endif

#if 0
#else
uint8_t OV2640_ReadReg(uint16_t Addr) {
    uint32_t timeout = DCMI_TIMEOUT_MAX;
    uint8_t Data = 0;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    /* Test on I2C2 EV5 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C2, OV2640_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

    /* Test on I2C2 EV6 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send I2C2 location address LSB */
    I2C_SendData(I2C2, (uint8_t)(Addr));

    /* Test on I2C2 EV8 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Clear AF flag if arised */
    I2C2->SR1 |= (uint16_t)0x0400;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    /* Test on I2C2 EV6 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C2, OV2640_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

    /* Test on I2C2 EV6 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Prepare an NACK for the next data received */
    I2C_AcknowledgeConfig(I2C2, DISABLE);

    /* Test on I2C2 EV7 and clear it */
    timeout = DCMI_TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Prepare Stop after receiving data */
    I2C_GenerateSTOP(I2C2, ENABLE);

    /* Receive the Data */
    Data = I2C_ReceiveData(I2C2);

    /* return the read data */
    return Data;
}
#endif

void OV2640_ReadID(OV2640_IDTypeDef *OV2640ID) {
    OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
    OV2640ID->Manufacturer_ID1 = OV2640_ReadReg(OV2640_SENSOR_MIDH);
    OV2640ID->Manufacturer_ID2 = OV2640_ReadReg(OV2640_SENSOR_MIDL);
    OV2640ID->PIDH = OV2640_ReadReg(OV2640_SENSOR_PIDH);
    OV2640ID->PIDL = OV2640_ReadReg(OV2640_SENSOR_PIDL);
}

void OV2640_Reset(void) {
    OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
    OV2640_WriteReg(OV2640_SENSOR_COM7, 0x80);
}

void OV2640_QQVGAConfig(void) {
    uint32_t i;

    OV2640_Reset();
    Delay(200);

    /* Initialize OV2640 */
    for ( i = 0; i < (sizeof(OV2640_QQVGA) / 2); i++) {
        OV2640_WriteReg(OV2640_QQVGA[i][0], OV2640_QQVGA[i][1]);
        Delay(2);
    }
}

void OV2640_QVGAConfig(void) {
    uint32_t i;

    OV2640_Reset();
    Delay(200);

    /* Initialize OV2640 */
    for ( i = 0; i < (sizeof(OV2640_QVGA) / 2); i++) {
        OV2640_WriteReg(OV2640_QVGA[i][0], OV2640_QVGA[i][1]);
        Delay(2);
    }
}

void OV2640_BrightnessConfig(uint8_t Brightness) {
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x04);
    OV2640_WriteReg(0x7c, 0x09);
    OV2640_WriteReg(0x7d, Brightness);
    OV2640_WriteReg(0x7d, 0x00);
}

void OV2640_BandWConfig(uint8_t BlackWhite) {
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, BlackWhite);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);
}

void OV2640_ColorEffectsConfig(uint8_t value1, uint8_t value2) {
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, value1);
    OV2640_WriteReg(0x7d, value2);
}

void OV2640_ContrastConfig(uint8_t value1, uint8_t value2) {
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x04);
    OV2640_WriteReg(0x7c, 0x07);
    OV2640_WriteReg(0x7d, 0x20);
    OV2640_WriteReg(0x7d, value1);
    OV2640_WriteReg(0x7d, value2);
    OV2640_WriteReg(0x7d, 0x06);
}
#endif

#endif  // OV2640_H_

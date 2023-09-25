#ifndef OVXXXX_FOO_H_
#define OVXXXX_FOO_H_

#include "SCCB.h"

uint8_t OVxxxx_WriteReg(uint8_t address,uint16_t Addr, uint8_t Data);
uint8_t OVxxxx_ReadReg(uint8_t address, uint16_t Addr);

uint8_t OVxxxx_WriteReg(uint8_t address,uint16_t Addr, uint8_t Data) {
 	StartI2C0();
	if (0 == I2CWrite0(address)) {
		StopI2C0();
		return 0;
	}
	delay_us(100);

  	if (0 == I2CWrite0(Addr)) {
		StopI2C0();
		return 0;
	}

	delay_us(100);
  	if(0 == I2CWrite0(Data)) {
		StopI2C0();
		return 0;
	}
  	StopI2C0();
	
  	return 1;
}

uint8_t OVxxxx_ReadReg(uint8_t address, uint16_t Addr) {
 	uint8_t regDat;
	StartI2C0();
	if(0 == I2CWrite0(address)) {
		StopI2C0();
		return 0;
	}
	delay_us(100);
  	if(0 == I2CWrite0(Addr)) {
		StopI2C0();
		return 0;
	}
	StopI2C0();
	delay_us(100);

	StartI2C0();
	if(0 == I2CWrite0(OV7725_DEVICE_READ_ADDRESS)) {
		StopI2C0();
		return 0;
	}
	delay_us(100);
  	regDat = I2CRead0();
  	NoAck0();
  	StopI2C0();
  	return regDat;
}


#endif  // OVXXXX_FOO_H

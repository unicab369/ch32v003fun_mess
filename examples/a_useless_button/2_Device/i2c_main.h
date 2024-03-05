#include "ch32v003fun.h"
#include "i2c_hal.h"

#define INA219_ADDR 0x40

//! BH1750
#define BH17_CONT_HI1 0x10      // 1 lux resolution 120ms
#define BH17_CONT_HI2 0x11      // .5 lux resolution 120ms
#define BH17_CONT_LOW 0x13      // 4 lux resolution 16ms
#define BH17_ONCE_HI1 0x20      // 1 lux resolution 120ms
#define BH17_ONCE_HI2 0x21      // .5 lux resolution 120ms
#define BH17_ONCE_LOW 0x23      // 4 lux resolution 16ms
#define BH17_RESET 0x07
#define BH17_POWER_ON 0x01
#define BH17_ADDR 0x23

uint16_t hexToDecimal(const uint8_t* hexArray, size_t length) {
	uint16_t result = 0;
	
	for (size_t i = 0; i < length; i++) {
		result = (result << 8) | hexArray[i];
	}
	
	return result;
}

void BH17_Setup() {
	uint8_t command[1] = { BH17_CONT_LOW };
	I2CWrite(BH17_ADDR, command, sizeof(command));
}

uint16_t BH17_Read() {
	uint8_t readData[2];
	I2CRead(BH17_ADDR, readData, 2);
	return hexToDecimal(readData, sizeof(readData));
}

uint16_t INA219_Read() {
	uint8_t readData[2];
	uint16_t value16;

	uint8_t data1[] = {0x02};
	I2CWrite(INA219_ADDR, data1, 2);
	I2CRead(INA219_ADDR, readData, 2);
	
	memcpy(&value16, readData, sizeof(value16));
	uint16_t swapped = (value16>>8) | (value16<<8);
	return (swapped>>3)*4;
}

//! SHT30
uint8_t SHT_RESET_CMD[2]        = { 0x30, 0xA2 };
uint8_t SHT_HIGHREP_HOLD_CMD[2] = { 0x2C, 0x06 };
uint8_t SHT_MEDREP_HOLD_CMD[2]  = { 0x2C, 0x0D };
uint8_t SHT_LOWREP_HOLD_CMD[2]  = { 0x2C, 0x10 };
uint8_t SHT_HIGHREP_FREE_CMD[2] = { 0x24, 0x00 };
uint8_t SHT_MEDREP_FREE_CMD[2]  = { 0x24, 0x0B };
uint8_t SHT_LOWREP_FREE_CMD[2]  = { 0x24, 0x16 };
uint8_t SHT_HEATER_DISABLE[2]   = { 0x30, 0x66 };
uint8_t SHT_HEATER_ENABLE[2]    = { 0x30, 0x6D };
#define SHT3_ADDR 0x44

struct SensorData {
   uint16_t tempF, hum, lux, voltage, mA;
};

void i2c_getReadings(struct SensorData* data) {
   // uint16_t lux = 0;
	// uint16_t tempC = 0, tempF = 0, hum = 0;
	// uint16_t busVoltage = 0, mA = 0;

   //! BH1750 Reading
   int check = I2CTest(BH17_ADDR);

   if (check) { 
      data->lux = BH17_Read(); 
   }

   // char strOut[32];
   // mini_snprintf(strOut, sizeof(strOut), "\nlux=%lu", lux);
   // printf(strOut);
   // ssd1306_drawstr_sz(0, 16, strOut, 1, fontsize_8x8);

   // //! SHT31 Reading
   int check2 = I2CTest(SHT3_ADDR);

   if (check2) {
      I2CWrite(SHT3_ADDR, SHT_LOWREP_FREE_CMD, sizeof(SHT_LOWREP_FREE_CMD));
      Delay_Ms(5);
      uint8_t buff[6];
      I2CRead(SHT3_ADDR, buff, 6);

      uint16_t rawTemp = 0, rawHum = 0;
      rawTemp = (buff[0]<<8) + buff[1];
      rawHum = (buff[3]<<8) + buff[4];

      // tempC = rawTemp*175/65535 - 45;
      data->tempF = rawTemp*63/13107 - 49;
      data->hum = rawHum*100/65535;
   }

   // mini_snprintf(strOut, sizeof(strOut), "\nt=%lu, h=%lu", tempF, hum);
   // printf(strOut);

   // //! INA219 Reading
   int check3 = I2CTest(INA219_ADDR);

   if (check3) {
      data->voltage = INA219_Read();
   }

   // printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", tempC, hum, lux, busVoltage, mA);
   // mini_snprintf(strOut, sizeof(strOut), "\nV=%lu, mA=%lu", busVoltage, mA);
   // printf(strOut);		

   // uint8_t allValues[] = { 0xB1, 0xCA, 0xFF, 0xFF, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00, 0x0D, 0x0A };
   // uint16_t allValues[] = { 0xCAB1, 0xFFFF, 0xFFFF, tempF, hum, lux, busVoltage, 0x0A0D };
}
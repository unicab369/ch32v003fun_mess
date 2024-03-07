/*
 * Example of a useless code. It doesn't do anything good except being great at what it does.
 * It demonstrates the power of repetition: if you repeat the samething many times over, it becomes you.
 * 
 * fun_gpio.h is based on ch32v_hal.h by Larry Bank
 * fun_timer.h is based on Example SysTick with IRQs work of E. Brombaugh and C. Lohr
 * by unicab369
 */

// .\minichlink.exe -u
// make all monitor

#include "ch32v003fun.h"
#include <stdio.h>
#include "1_Foundation/fun_button.h"
#include "2_Device/i2c_main.h"
#include "3_Nrf/nrf24l01_main.h"

// #define PIN_LOWPOWER 0xD3
#define PIN_LOWPOWER 0xD4
#define PIN_BUTTON 0xD2
// #define PIN_LED 0xA2
#define PIN_LED 0xD0
#define PIN_MODE 0xA1
#define PIN_DONE 0xC3

void nrf_onReceive(uint8_t* data) {
	digitalWrite(PIN_LED, !digitalRead(PIN_LED));
	struct SensorData* receivedData = (struct SensorData*) data;

	printf("\n\rRecieved");
	printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", 
				receivedData->tempF, receivedData->hum, receivedData->lux, receivedData->voltage, receivedData->mA);
}

//######### Button

void button_onSingleClick() {
   printf("\nI'M USELESS.");
	digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void button_onDoubleClick() {
   printf("\nI'M USELESS TWICE.");
}

void make_inputPullups() {
	// Set all GPIOs to input pull up
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

	// GPIOA: Set to output
	GPIOA->CFGLR = (GPIO_CNF_IN_PUPD<<(4*2)) | (GPIO_CNF_IN_PUPD<<(4*1));
	GPIOA->BSHR = GPIO_BSHR_BS2 | GPIO_BSHR_BR1;

	GPIOC->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
					(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
					(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
					(GPIO_CNF_IN_PUPD<<(4*1)) | (GPIO_CNF_IN_PUPD<<(4*0));
	GPIOC->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 | GPIO_BSHR_BS4 |
					GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS1 | GPIO_BSHR_BS0;

	GPIOD->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
					(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
					(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
					(GPIO_CNF_IN_PUPD<<(4*0));
	GPIOD->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 | GPIO_BSHR_BS4 |
					GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS0;
	
}

void sleepMode_setup(uint8_t useButton) {
	// enable power interface module clock
	RCC->APB1PCENR |= RCC_APB1Periph_PWR;

	// enable low speed oscillator (LSI)
	RCC->RSTSCKR |= RCC_LSION;
	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

	// enable AutoWakeUp event
	EXTI->EVENR |= EXTI_Line9;
	EXTI->FTENR |= EXTI_Line9;

	//# t = AWUWR*AWUPSC/fLSI
	// fLSI = 128000
	// AWUWR = 1 to 63
	// AWUPSC = 2048, 4096, 10240 or 61440, though lower values are possible.    

	PWR->AWUPSC |= PWR_AWU_Prescaler_4096;		// configure AWU prescaler
	PWR->AWUWR &= ~0x3f; PWR->AWUWR |= 63;		// configure AWU window comparison value
	PWR->AWUCSR |= (1 << 1);						// enable AWU

	//# standby_btn
	if (useButton) {
		RCC->APB2PCENR |= RCC_AFIOEN;							// AFIO is needed for EXTI
		AFIO->EXTICR |= (uint32_t)(0b11 << (2*2));		// assign pin 2 interrupt from portD (0b11) to EXTI channel 2

		// enable line2 interrupt event
		EXTI->EVENR |= EXTI_Line2;
		EXTI->FTENR |= EXTI_Line2;
	}

	//# sleep
	PWR->CTLR |= PWR_CTLR_PDDS;					// select standby on power-down	
	PFIC->SCTLR |= (1 << 2);						// peripheral interrupt controller send to deep sleep
}

//######### MAIN
int main() {  
	SystemInit();
	
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_LOWPOWER, INPUT_PULLUP);
	pinMode(PIN_MODE, INPUT_PULLUP);
	pinMode(0xC3, OUTPUT);
	pinMode(PIN_DONE, OUTPUT);

	I2CInit(0xC1, 0xC2, 100000);
	BH17_Setup();

	// button_setup(PIN_BUTTON);

	// lowPower flag INPUT_PULLUP, Low for OFF
	uint8_t readLowpower = digitalRead(PIN_LOWPOWER);
	struct SensorData readings = { 0, 0, 0, 0, 0 };

	// readLowpower == 1
	if (readLowpower == 0) {
		printf("\n\rIM HERE 1111");
		nrf_setup(0);

		for(;;) {
			i2c_getReadings(&readings);
			printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", 
						readings.tempF, readings.hum, readings.lux, readings.voltage, readings.mA);
			sendData(&readings, sizeof(readings));
			digitalWrite(PIN_LED, !digitalRead(PIN_LED));
			digitalWrite(PIN_DONE, 1);
			Delay_Ms(2000);
		}
	} else {
		printf("\n\rIM HERE 2222");
		nrf_setup(1);

		// GPIO D0 Push-Pull for RX notification
		RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
		GPIOD->CFGLR &= ~(0xf<<(4*4));
		GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

		for(;;) {
			nrf_run(1);
		}
	}

	// if (readLowpower == 1) {
	// 	Delay_Ms(3500);
	// 	make_inputPullups();
	// 	sleepMode_setup(1);

	// 	for(;;) {
	// 		__WFE();
	// 		SystemInit();

	// 		digitalWrite(0xC3, 0);
	// 		nrf_setup(0);
	// 		send();
	// 		send();
	// 		digitalWrite(0xC3, 1);
	// 		digitalWrite(PIN_DONE, 1);
	// 		// Delay_Ms(1000);
	// 		// // make_inputPullups();
	// 		// // pinMode(0xC3, INPUT_PULLDOWN);
	// 	}
	// } 
	// else {
	// 	uint8_t readMode = digitalRead(PIN_MODE);
	// 	printf("\nMOde=%u", readMode);
	// 	nrf_setup(readMode);

	// 	// GPIO D0 Push-Pull for RX notification
	// 	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
	// 	GPIOD->CFGLR &= ~(0xf<<(4*4));
	// 	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

	// 	for(;;) {
	// 		nrf_run(readMode);
	// 		button_run();
	// 	}
	// }
}


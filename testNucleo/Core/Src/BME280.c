/*
 * BME280.c
 *
 *  Created on: Oct 12, 2020
 *      Author: René
 */
#include "BME280.h"

static int32_t t_fine;

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

static uint8_t bufI2C[20];

// From BME280 datasheet
int32_t BME280_compensate_T(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2))>>11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) -  ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1+var2;
	T = (t_fine *5 + 128) >> 8;
	return T;
}

/**
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format
 * (24 integer bits and 8 fractional bits)
 * Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * Warning: the BME280_compensate_T function must have been called before calling this function
 */
uint32_t BME280_compensate_P_int64(int32_t adc_P) {
	int64_t var1, var2, p;
	var1=((int64_t)t_fine)-128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if(var1 == 0) {
		return 0; // avoid exception caused by division by 0
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

void BME280_get_compensation_data(void) {
	bufI2C[0] = 0x88; // "dig_Tx" registers
	  HAL_I2C_Master_Transmit(&hi2c1, BME_ADDR, bufI2C, 1, HAL_MAX_DELAY); //send 1 bytes

	  HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, bufI2C, 6, HAL_MAX_DELAY); //receive 6 bytes
	  dig_T1 = bufI2C[0] | (uint16_t)bufI2C[1] << 8;
	  dig_T2 = bufI2C[2] | (uint16_t)bufI2C[3] << 8;
	  dig_T3 = bufI2C[4] | (uint16_t)bufI2C[5] << 8;
	  HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, bufI2C, 18, HAL_MAX_DELAY); //receive 18 bytes
	  dig_P1 = bufI2C[0] | (uint16_t)bufI2C[1] << 8;
	  dig_P2 = bufI2C[2] | (uint16_t)bufI2C[3] << 8;
	  dig_P3 = bufI2C[4] | (uint16_t)bufI2C[5] << 8;
	  dig_P4 = bufI2C[6] | (uint16_t)bufI2C[7] << 8;
	  dig_P5 = bufI2C[8] | (uint16_t)bufI2C[9] << 8;
	  dig_P6 = bufI2C[10] | (uint16_t)bufI2C[11] << 8;
	  dig_P7 = bufI2C[12] | (uint16_t)bufI2C[13] << 8;
	  dig_P8 = bufI2C[14] | (uint16_t)bufI2C[15] << 8;
	  dig_P9 = bufI2C[16] | (uint16_t)bufI2C[17] << 8;
	  return;
}

void BME280_init(uint8_t ossr_t, uint8_t ossr_p, uint8_t mode, uint8_t period){
	 bufI2C[0] = 0xD0; // "id" register
	  HAL_I2C_Master_Transmit(&hi2c1, BME_ADDR, bufI2C, 1, HAL_MAX_DELAY); //send 1 byte
	  HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, bufI2C, 1, HAL_MAX_DELAY); //receive 1 byte

	  if(bufI2C[0] != 0x60){ // if the device id does not match expected
	  	  Error_Handler();
	  }

	  uint8_t mask3 = 0b111;
	  uint8_t mask2 = 0b11;
	  bufI2C[0] = 0xF4; // "ctrl_meas" register
	  bufI2C[1] = (ossr_t & mask3) << 5 | (ossr_p & mask3) << 2 | (mode & mask2);
	  bufI2C[2] = (period & mask3) << 5; // 1 measure every 500ms
	  HAL_I2C_Master_Transmit(&hi2c1, BME_ADDR, bufI2C, 3, HAL_MAX_DELAY); //send 3 bytes

	  BME280_get_compensation_data();

	  return;
}

void BME280_get_measurements(struct BME280_data_t* out){
	int32_t adc_T;
	int32_t adc_P;
	bufI2C[0] = 0xF7; // temperature MSB register
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDR, bufI2C, 1, HAL_MAX_DELAY); // send 1 byte

	HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, bufI2C, 3, HAL_MAX_DELAY); // receive 3 bytes
	adc_P = (int32_t)bufI2C[0] << 12 | (int32_t)bufI2C[1] << 4 | bufI2C[2] >> 4;
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, bufI2C, 3, HAL_MAX_DELAY); // receive 3 bytes
	adc_T = (int32_t)bufI2C[0] << 12 | (int32_t)bufI2C[1] << 4 | bufI2C[2] >> 4;

	out->T = BME280_compensate_T(adc_T)/100.0;
	out->P = BME280_compensate_P_int64(adc_P)/100.0/256.0;
	out->H = 0;

	return;
}

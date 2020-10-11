/*
 * BME280.h
 *
 *  Created on: Oct 12, 2020
 *      Author: René
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"

#define BME_ADDR 0b11101100


#define BME280_OVERSAMPLING_1	0b001
#define BME280_OVERSAMPLING_2	0b010
#define BME280_OVERSAMPLING_4	0b011
#define BME280_OVERSAMPLING_8	0b100
#define BME280_OVERSAMPLING_16	0b101
#define BME280_DISABLED			0b000

#define BME280_SLEEP_MODE	0b00
#define BME280_FORCED_MODE	0b01
#define BME280_NORMAL_MODE	0b11

#define BME280_RATE_500_MS	0b100

struct BME280_data_t {
	float T;
	float P;
	float H;
};

int32_t BME280_compensate_T(int32_t);
uint32_t BME280_compensate_P_int64(int32_t);
void BME280_get_compensation_data(void);
void BME280_init(uint8_t ossr_t, uint8_t ossr_p, uint8_t mode, uint8_t period);
void BME280_get_measurements(struct BME280_data_t*);

#endif /* INC_BME280_H_ */

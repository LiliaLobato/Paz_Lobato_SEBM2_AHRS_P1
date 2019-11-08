/*
 * BMI160.h
 *
 *  Created on: Oct 28, 2019
 *      Author: gabriel
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "rtos_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdint.h"

#define BMI160_SLAVE_ADDRESS 0b1101000 //BMI160 PHYSICAL ADDRESS
#define BMI160_CMD_REGISTER 0x7E //REGISTER TO COMUNICATE WITH PMU REGISTER
#define BMI160_ACC_NORMAL_MODE 0x11 //CMD_REG = ACCELEROMETER NORMAL MODE VALUE
#define BMI160_GYRO_NORMAL_MODE 0x15 //CMD_REG = GYROSCOPE NORMAL MODE VALUE
#define BMI160_GYRO_X_LSB_REG 0x0C
#define BMI160_GYRO_X_MSB_REG 0x0D
#define BMI160_GYRO_Y_LSB_REG 0x0E
#define BMI160_GYRO_Y_MSB_REG 0x0F
#define BMI160_GYRO_Z_LSB_REG 0x10
#define BMI160_GYRO_Z_MSB_REG 0x11
#define BMI160_ACC_X_LSB_REG 0x12
#define BMI160_ACC_X_MSB_REG 0x13
#define BMI160_ACC_Y_LSB_REG 0x14
#define BMI160_ACC_Y_MSB_REG 0x15
#define BMI160_ACC_Z_LSB_REG 0x16
#define BMI160_ACC_Z_MSB_REG 0x17
#define K64F_I2C_NUMBER_OF_CHANNELS 3
#define K64F_I2C0_SCL_PIN 2 //PTB
#define K64F_I2C0_SDA_PIN 3
#define K64F_I2C1_SCL_PIN 10 //PTC
#define K64F_I2C1_SDA_PIN 11
#define K64F_I2C2_SCL_PIN 11 //PTA
#define K64F_I2C2_SDA_PIN 12
#define BAUDRATE_FOR_I2C 115200U
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_MASTER_BASEADDR I2C0
#define I2C_MAX_DELAY 100 //100ms

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U
#define I2C__INIT_DATA_LENGTH 1U
#define I2C__INIT_DATA_LENGTH 1U

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}BMI160_data_struct_t;



void BMI160_init();
void BMI160_I2C_config();
BMI160_data_struct_t BMI160_get_ACC();
BMI160_data_struct_t BMI160_get_GYRO();

#endif /* BMI160_H_ */

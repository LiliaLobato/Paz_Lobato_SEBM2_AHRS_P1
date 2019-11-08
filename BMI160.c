/*
 * BMI160.c
 *
 *  Created on: Oct 28, 2019
 *      Author: gabriel
 */

#include "BMI160.h"
#include "stdint.h"


void BMI160_init()
{
	BMI160_I2C_config();

	uint8_t ACC_init_data;
	uint8_t GYRO_init_data;
	rtos_i2c_flag_t transfer_status = rtos_i2c_fail;

	ACC_init_data = BMI160_ACC_NORMAL_MODE;
	GYRO_init_data = BMI160_GYRO_NORMAL_MODE;

	//Accelerometer configuration
	transfer_status = rtos_i2c_transfer(rtos_i2c_0, &ACC_init_data, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_CMD_REGISTER, 1);

	vTaskDelay(pdMS_TO_TICKS(I2C_MAX_DELAY));

	//Gyroscope configuration
	transfer_status = rtos_i2c_transfer(rtos_i2c_0, &GYRO_init_data, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_CMD_REGISTER, 1);
	//vTaskSuspend()
	BMI160_data_struct_t data;
	//data = BMI160_get_ACC();
	data = BMI160_get_GYRO();
}


void BMI160_I2C_config()
{
	rtos_i2c_config_t I2C_RTOS_config;

	NVIC_SetPriority(I2C0_IRQn,5);

	CLOCK_EnableClock(kCLOCK_PortB); //I2C0 in Port B

	//I2C_MasterGetDefaultConfig(&I2C_RTOS_config);
	I2C_RTOS_config.SCL_pin =  K64F_I2C0_SCL_PIN;
	I2C_RTOS_config.SDA_pin = K64F_I2C0_SDA_PIN;
	I2C_RTOS_config.baudrate = BAUDRATE_FOR_I2C;
	I2C_RTOS_config.i2c_number = rtos_i2c_0;
	I2C_RTOS_config.port = rtos_i2c_portB;
	I2C_RTOS_config.pin_mux = kPORT_MuxAlt2;

	rtos_i2c_init(I2C_RTOS_config);


}

BMI160_data_struct_t BMI160_get_ACC()
{
	BMI160_data_struct_t ACC_data;
	int8_t MSB = 0;
	int8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_X_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_X_LSB_REG, 1);
	ACC_data.x = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Y
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_Y_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_Y_LSB_REG, 1);
	ACC_data.y = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Z
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_Z_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_Z_LSB_REG, 1);
	ACC_data.z = (MSB << 8) + LSB;

	return ACC_data;

}

BMI160_data_struct_t BMI160_get_GYRO()
{
	BMI160_data_struct_t GYRO_data;
	int8_t MSB = 0;
	int8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_X_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_X_LSB_REG, 1);
	GYRO_data.x = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Y
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_Y_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_Y_LSB_REG, 1);
	GYRO_data.y = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Z
	rtos_i2c_receive(rtos_i2c_0, &MSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_Z_MSB_REG, 1);
	rtos_i2c_receive(rtos_i2c_0, &LSB, I2C__INIT_DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_Z_LSB_REG, 1);
	GYRO_data.z = (MSB << 8) + LSB;

	return GYRO_data;

}

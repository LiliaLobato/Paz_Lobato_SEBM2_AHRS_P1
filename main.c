/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    P1_Escuadron_Lobo.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOSConfig.h"
#include "BMI160.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mahony.h"
#include "fsl_uart_freertos.h"
#include "BMI160.h"

#define GYR_ACOND 360/32768 // (2.0f/((float)(1<<15)))
#define ACC_ACOND 1 // (2.0f/((float)(1<<15)))
#define AHRS_SAMPLES_NUMBER 100

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
void Taskeishon(void *PvParameters);
/*
 * @brief   Application entry point.
 */
typedef struct{
	uint32_t header;
	float x;
	float  y;
	float  z;
}msg_t;

typedef struct
{
	float x;
	float y;
	float z;
}estadistica_t;

TaskHandle_t xHandle;
BMI160_data_struct_t acc;
BMI160_data_struct_t gyr;
BMI160_data_struct_t values_acc;
BMI160_data_struct_t values_gyr;
BMI160_data_struct_t offset_acc;
BMI160_data_struct_t offset_gyr;
BMI160_data_struct_t data;
estadistica_t std;
estadistica_t media;
estadistica_t varianza;
estadistica_t desviacion_estandar;
msg_t uart_msg;

void Task(void *pvParameters);

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    xHandle = xTaskCreate(Task, "Task", 200, NULL, 5, NULL);

    vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}

void Task(void *PvParameters)
{
		BMI160_init();
		uart_rtos_handle_t handle_UART;
		uart_handle_t t_handle_UART;
		uart_rtos_config_t config_UART;
		offset_gyr.x = 0;
		offset_gyr.y = 0;
		offset_gyr.z = 2;
		offset_acc.x = 0;
		offset_acc.y = 0;
		offset_acc.z = 0;

	    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
	    // Configuring the UART configuration struct.
	        config_UART.base = UART0_BASE;
	        config_UART.srcclk = 120000000;
	        config_UART.baudrate = 115200;
	        config_UART.parity = 0;
	        config_UART.stopbits = 0;

	    MahonyAHRSEuler_t mahony;

for(uint8_t i = 0; i<=AHRS_SAMPLES_NUMBER;i++) {
	data = BMI160_get_ACC();
	media.x = media.x + ((float)data.x * (float)data.x);
	std.x = std.x + (float)data.x;
	media.y = media.y + ((float)data.y * (float)data.y);
	std.y = std.y + (float)data.y;
	media.z = media.z + ((float)data.z * (float)data.z);
	std.z = std.z + (float)data.z;
}
media.x = media.x / AHRS_SAMPLES_NUMBER;
media.y = media.y / AHRS_SAMPLES_NUMBER;
media.z = media.z / AHRS_SAMPLES_NUMBER;
std.x = (std.x * std.x) / AHRS_SAMPLES_NUMBER;
std.y = (std.y * std.y) / AHRS_SAMPLES_NUMBER;
std.z = (std.z * std.z) / AHRS_SAMPLES_NUMBER;
varianza.x = media.x - std.x;
varianza.y = media.y - std.y;
varianza.z = media.z - std.z;
desviacion_estandar.x = sqrt(varianza.x);
desviacion_estandar.y = sqrt(varianza.y);
desviacion_estandar.z = sqrt(varianza.z);
printf("acc: x = %i, y = %i, z = %i\n\r",(uint16_t)varianza.x, (uint16_t)varianza.y, (uint16_t)varianza.z);
printf("desviacion: x = %i, y = %i, z = %i\n\r",desviacion_estandar.x, desviacion_estandar.y, desviacion_estandar.z);

for(uint8_t i = 0; i<AHRS_SAMPLES_NUMBER;i++) {
	data = BMI160_get_GYRO();
	media.x = media.x + (data.x * data.x);
	std.x = std.x + data.x;
	media.y = media.y + (data.y * data.y);
	std.y = std.y + data.y;
	media.z = media.z + (data.z * data.z);
	std.z = std.z + data.z;
}
media.x = media.x / AHRS_SAMPLES_NUMBER;
media.y = media.y / AHRS_SAMPLES_NUMBER;
media.z = media.z / AHRS_SAMPLES_NUMBER;
std.x = (std.x * std.x) / AHRS_SAMPLES_NUMBER;
std.y = (std.y * std.y) / AHRS_SAMPLES_NUMBER;
std.z = (std.z * std.z) / AHRS_SAMPLES_NUMBER;
varianza.x = media.x - std.x;
varianza.y = media.y - std.y;
varianza.z = media.z - std.z;
desviacion_estandar.x = sqrt(varianza.x);
desviacion_estandar.y = sqrt(varianza.y);
desviacion_estandar.z = sqrt(varianza.z);
printf("desviacion: x = %i, y = %i, z = %i\n\r",desviacion_estandar.x, desviacion_estandar.y, desviacion_estandar.z);

		while(1)
		{
			acc = BMI160_get_ACC();
			gyr = BMI160_get_GYRO();
			//printf("acc: x = %i, y = %i, z = %i\n\r",acc.x, acc.y, acc.z);
			//printf("gyr: x = %i, y = %i, z = %i\n\r",gyr.x, gyr.y, gyr.z);
			values_gyr.x = (float)gyr.x*GYR_ACOND + offset_gyr.x;
			values_gyr.y = (float)gyr.y*GYR_ACOND + offset_gyr.y;
			values_gyr.z = (float)gyr.z*GYR_ACOND + offset_gyr.z;
			values_acc.x = (float)acc.x*ACC_ACOND + offset_acc.x;
			values_acc.y = (float)acc.y*ACC_ACOND + offset_acc.y;
			values_acc.z = (float)acc.z*ACC_ACOND + offset_acc.z;
			//printf("acc: x = %i, y = %i, z = %i\n\r",values_acc.x, values_acc.y, values_acc.z);
			//printf("gyr: x = %i, y = %i, z = %i\n\r",values_gyr.x, values_gyr.y, values_gyr.z);


			mahony = MahonyAHRSupdateIMU(values_gyr.x, values_gyr.y, values_gyr.z, values_acc.x, values_acc.y, values_acc.z);
			uart_msg.header = 0xAAAAAAAA;
			uart_msg.x=-mahony.pitch;
			uart_msg.y=-mahony.roll;
			uart_msg.z=-mahony.yaw;
			//printf("uart: x = %i, y = %i, z = %i\n\r",uart_msg.x, uart_msg.y, uart_msg.z);
			uint8_t * pt_uart =(uint8_t *) &uart_msg;
			 config_UART.buffer = pt_uart ;
			 config_UART.buffer_size = sizeof(uart_msg);
			    if(0 != UART_RTOS_Init(&handle_UART,&t_handle_UART,&config_UART)){
			        	    	while(1);
			        	    }
			UART_RTOS_Send(&handle_UART,(uint8_t *)config_UART.buffer,config_UART.buffer_size);
			UART_RTOS_Deinit(&handle_UART);
			vTaskDelay(1);
		}
}

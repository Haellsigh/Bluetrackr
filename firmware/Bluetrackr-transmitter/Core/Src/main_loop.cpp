#include "main_loop.h"
#include "main.h"

#include "delay.h"

#include "mpu.h"

void main_loop() {
	// Initializes delay library
	blt::delayInit();

	MPU9250_CONFIG_t cfg;
	cfg.ACCEL_SCALE = ACCEL_SCALE_16G;
	cfg.GYRO_SCALE = GYRO_SCALE_2000dps;
	cfg.GPIOx = GPIOB;
	cfg.GPIO_PIN = MPU_CS_Pin;
	cfg.hspi = &hspi1;

	HAL_GPIO_WritePin(useled_GPIO_Port, useled_Pin, GPIO_PIN_SET);

	if (MPU9250_Config(&cfg) != MPU9250_RESULT_OK) {
		while (1) {
			HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
			HAL_Delay(400);
		}
	}
	if (MPU9250_Initialize(&cfg) != MPU9250_RESULT_OK) {
		while (1) {
			HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
			HAL_Delay(50);
		}
	}
	if (MPU9250_Calibrate(&cfg) != MPU9250_RESULT_OK) {
		while (1) {
			HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
			HAL_Delay(1000);
		}
	}

	MPU9250_DATA_t data;
	int8_t datastr[30] = "";
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		MPU9250_Update7DOF(&cfg, &data);

		sprintf(datastr, "%6.2f, %6.2f, %6.2f\r\n", data.gyro[0], data.gyro[1],
				data.gyro[2]);

		HAL_UART_Transmit(&huart1, datastr, strlen(datastr), 0xFFFF);
		HAL_Delay(50);
	}

	while (true) {
	}
}

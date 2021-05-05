/*
 * Copyright © 2021 - present | uss.c by Javinator9889
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see https://www.gnu.org/licenses/.
 *
 * Created by Javinator9889 on 26/03/21 - uss.c.
 */
#include "uss.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <stm32f4xx_hal.h>
#include "dwt_stm32_delay.h"

/**
 * @brief Reads the measured distance from the ultrasonic sensor.
 * 
 * @return uint32_t - the measured distance, in meters.
 */
uint32_t USS_read_distance(void) {
	__IO uint8_t flag = 0;
	__IO uint32_t disTime = 0;
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
	
	while(flag == 0) {
		while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET) {
			disTime++;
			flag = 1;
		}
	}
	return disTime;
}
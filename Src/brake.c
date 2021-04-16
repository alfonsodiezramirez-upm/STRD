/*
 * Copyright © 2021 - present | brake.c by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - brake.c.
 */
#include "brake.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <stdbool.h>

static SemaphoreHandle_t BRAKE_sem = NULL;
bool BRAKE_active = false;

void BRAKE_init(void) {
    BRAKE_sem = LOCK_create(NULL);
}

bool BRAKE_lock(void) {
    /*configASSERT(BRAKE_sem != NULL);
    while (BRAKE_active);
    
    return xSemaphoreTake(BRAKE_sem, 0) == pdTRUE;*/
	BRAKE_active = 0;
	return true;
}

bool BRAKE_wait(void) {
    configASSERT(BRAKE_sem != NULL);
    //while (!BRAKE_active);

    return xSemaphoreTake(BRAKE_sem, portMAX_DELAY) == pdTRUE;
}

bool BRAKE_set(void) {
		configASSERT(BRAKE_sem != NULL);
    return true;//xSemaphoreGive(BRAKE_sem) == pdTRUE;
}

bool BRAKE_free(void) {
    return true;
}

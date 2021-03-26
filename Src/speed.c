/*
 * Copyright © 2021 - present | locki_h by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - locki_h.
 */
#include "speed.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

static SemaphoreHandle_t SPEED_sem = NULL;
static int SPEED_speed = 0;

void SPEED_init(void) {
    SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();
    configASSERT(xSemaphore != NULL);
    configASSERT(xSemaphoreGive(xSemaphore) != pdTRUE);
    SPEED_sem = xSemaphore;
}

void SPEED_set(int speed) {
    configASSERT(SPEED_sem != NULL);
    if (xSemaphoreTake(SPEED_sem, portMAX_DELAY) == pdTRUE) {
        SPEED_speed = speed;
        xSemaphoreGive(SPEED_sem);
    }
}

int SPEED_get(void) {
    return SPEED_speed;
}

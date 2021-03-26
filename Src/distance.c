/*
 * Copyright © 2021 - present | distance.c by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - distance.c.
 */
#include "distance.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

static SemaphoreHandle_t DISTANCE_sem = NULL;
static float DISTANCE_distance = 0;

void DISTANCE_init(void) {
    DISTANCE_sem = LOCK_create();
}

void DISTANCE_set(float distance) {
    configASSERT(DISTANCE_sem != NULL);
    if (xSemaphoreTake(DISTANCE_sem, portMAX_DELAY) == pdTRUE) {
        DISTANCE_distance = distance;
        xSemaphoreGive(DISTANCE_sem);
    }
}

float DISTANCE_get(void) {
    float distance = -1;
    configASSERT(DISTANCE_sem != NULL);
    if (xSemaphoreTake(DISTANCE_sem, portMAX_DELAY) == pdTRUE) {
        distance = DISTANCE_distance;
        xSemaphoreGive(DISTANCE_sem);
    }
    return distance;
}

void DISTANCE_delete(void) {
    LOCK_destroy(DISTANCE_sem);
    DISTANCE_sem = NULL;
    DISTANCE_distance = 0;
}

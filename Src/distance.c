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

static SemaphoreHandle_t INSTANCE_sem = NULL;
static volatile float DISTANCE_distance = 0;
static volatile int BRAKE_intensity = 0;

void DISTANCE_init(void) {
    INSTANCE_sem = LOCK_create(NULL);
}

void DISTANCE_set(float distance) {
    configASSERT(INSTANCE_sem != NULL);
    if (xSemaphoreTake(INSTANCE_sem, portMAX_DELAY) == pdTRUE) {
        DISTANCE_distance = distance;
        xSemaphoreGive(INSTANCE_sem);
    }
}

float DISTANCE_get(void) {
    float distance = -1;
    configASSERT(INSTANCE_sem != NULL);
    if (xSemaphoreTake(INSTANCE_sem, portMAX_DELAY) == pdTRUE) {
        distance = DISTANCE_distance;
        xSemaphoreGive(INSTANCE_sem);
    }
    return distance;
}

void DISTANCE_delete(void) {
    LOCK_destroy(INSTANCE_sem);
    INSTANCE_sem = NULL;
    DISTANCE_distance = 0;
}

void BRAKE_intensity_set(int intensity) {
    configASSERT(INSTANCE_sem != NULL);
    if (xSemaphoreTake(INSTANCE_sem, portMAX_DELAY) == pdTRUE) {
        BRAKE_intensity = intensity;
        xSemaphoreGive(INSTANCE_sem);
    }
}

int BRAKE_intensity_get(void) {
    int intensity = -1;
    configASSERT(INSTANCE_sem != NULL);
    if (xSemaphoreTake(INSTANCE_sem, portMAX_DELAY) == pdTRUE) {
        intensity = BRAKE_intensity;
        xSemaphoreGive(INSTANCE_sem);
    }
    return intensity;
}
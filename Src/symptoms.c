/*
 * Copyright © 2021 - present | symptomps.h by Javinator9889
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
 * Created by Javinator9889 on 26/03/21 - symptoms.h.
 */
#include "symptoms.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <stdbool.h>

static SemaphoreHandle_t SYMPTOMS1_sem = NULL;
static SemaphoreHandle_t SYMPTOMS2_sem = NULL;

static float GIROSCOPE_x = .0F;
static float GIROSCOPE_y = .0F;
static float GIROSCOPE_z = .0F;

static int WHEEL_position = 0;
static bool WHEEL_status_grab = false;

void SYMPTOMS_init(void) {
    SYMPTOMS1_sem = LOCK_create(NULL);
    SYMPTOMS2_sem = LOCK_create(NULL);
}

void GIROSCOPE_set(float x, float y, float z) {
    configASSERT(SYMPTOMS1_sem != NULL);
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        GIROSCOPE_x = x;
        GIROSCOPE_y = y;
        GIROSCOPE_z = z;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
    
}
float GIROSCOPE_get_X(void) {
    configASSERT(SYMPTOMS1_sem != NULL);
    float x = -1;
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        x = GIROSCOPE_x;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
    return x;
}

float GIROSCOPE_get_Y(void) {
    configASSERT(SYMPTOMS1_sem != NULL);
    float y = -1;
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        y = GIROSCOPE_y;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
    return y;
}

float GIROSCOPE_get_Z(void) {
    configASSERT(SYMPTOMS1_sem != NULL);
    float z = -1;
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        z = GIROSCOPE_z;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
    return z;
}

void WHEEL_set(int position) {
    configASSERT(SYMPTOMS1_sem != NULL);
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        WHEEL_position = position;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
}

int WHEEL_get(void) {
    configASSERT(SYMPTOMS1_sem != NULL);
    int position = -1;
    if (xSemaphoreTake(SYMPTOMS1_sem, portMAX_DELAY) == pdTRUE) {
        position = WHEEL_position;
        xSemaphoreGive(SYMPTOMS1_sem);
    }
    return position;
}

void WHEEL_grab(bool is_grabbed) {
    configASSERT(SYMPTOMS2_sem != NULL);
    if (xSemaphoreTake(SYMPTOMS2_sem, portMAX_DELAY) == pdTRUE) {
        WHEEL_status_grab = is_grabbed;
        xSemaphoreGive(SYMPTOMS2_sem);
    }
}

bool WHEEL_is_grabbed(void) {
    configASSERT(SYMPTOMS2_sem != NULL);
    bool is_grabbed = false;
    if (xSemaphoreTake(SYMPTOMS2_sem, portMAX_DELAY) == pdTRUE) {
        is_grabbed = WHEEL_status_grab;
        xSemaphoreGive(SYMPTOMS2_sem);
    }
    return is_grabbed;
}

void SYMPTOMS_delete(void) {
    LOCK_destroy(SYMPTOMS1_sem);
    LOCK_destroy(SYMPTOMS2_sem);

    SYMPTOMS1_sem = NULL;
    SYMPTOMS2_sem = NULL;

    GIROSCOPE_x = .0F;
    GIROSCOPE_y = .0F;
    GIROSCOPE_z = .0F;

    WHEEL_position = 0;
    WHEEL_status_grab = false;
}
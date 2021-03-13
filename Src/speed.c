/*
 * Copyright © 2021 - present | speed.h by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - speed.h.
 */
#include "speed.h"
#include <lock.h>

static SemaphoreHandle_t SPEED_sem = NULL;
static int SPEED_speed = 0;

void SPEED_init(void) {
    SPEED_sem = LOCK_create();
}

void SPEED_set(int speed) {
    configASSERT(SPEED_sem != NULL);
    if (xSemaphoreTake(SPEED_sem, portMAX_DELAY) == pdTRUE) {
        SPEED_speed = speed;
        xSemaphoreGive(SPEED_sem);
    }
}

int SPEED_get(void) {
    int speed = -1;
    configASSERT(SPEED_sem != NULL);
    if (xSemaphoreTake(SPEED_sem, portMAX_DELAY) == pdTRUE) {
        speed = SPEED_speed;
        xSemaphoreGive(SPEED_sem);
    }
    return speed;
}

void SPEED_delete(void) {
    LOCK_destroy(SPEED_sem);
    SPEED_sem = NULL;
    SPEED_speed = 0;
}

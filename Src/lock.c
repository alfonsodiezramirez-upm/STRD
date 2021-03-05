/*
 * Copyright © 2021 - present | lock_c by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - lock_c.
 */
#include "lock.h"
#include <portmacro.h>
#include <projdefs.h>
#include <stddef.h>
#include <stdlib.h>


SemaphoreHandle_t LOCK_create(void) {
    SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore != NULL) {
        return xSemaphore;
    }
    return NULL;
}

void LOCK_destroy(SemaphoreHandle_t semaforo) {
    vSemaphoreDelete(semaforo);
}

long LOCK_acquire(SemaphoreHandle_t semaforo) {
    return xSemaphoreTake(semaforo, portMAX_DELAY);
}

void LOCK_release(SemaphoreHandle_t semaforo) {
    xSemaphoreGive(semaforo);
}

// void *LOCK_read(pobject_t *container) {
//    if (xSemaphoreTake(container->lock, portMAX_DELAY) == pdTRUE) {
//         void *dest = container->data;
//         xSemaphoreGive(container->lock);
//         return dest;
//     } else {
//         return NULL;
//     }
// }

// char LOCK_write(pobject_t *container, void *value) {
//     if (xSemaphoreTake(container->lock, portMAX_DELAY) == pdTRUE) {
//         container->data = value;
//         xSemaphoreGive(container->lock);
//         return 0;
//     } else {
//         return 1;
//     }
// }

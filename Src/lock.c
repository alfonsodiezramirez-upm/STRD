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
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include "task.h"


SemaphoreHandle_t LOCK_create(StaticSemaphore_t *mutexBuffer) {
    SemaphoreHandle_t xSemaphore = NULL;
    BaseType_t xReturned;

    #if defined(configSUPPORT_STATIC_ALLOCATION) && (configSUPPORT_STATIC_ALLOCATION == 1)
    if (mutexBuffer != NULL) xSemaphore = xSemaphoreCreateMutexStatic(mutexBuffer);
    else
    #endif
    xSemaphore = xSemaphoreCreateMutex();

    configASSERT(xSemaphore != NULL);    
    configASSERT(xSemaphoreGive(xSemaphore) != pdTRUE);
    
    return xSemaphore;
}

inline void LOCK_destroy(SemaphoreHandle_t sem) {
    vSemaphoreDelete(sem);
}

inline long LOCK_acquire(SemaphoreHandle_t sem) {
    return xSemaphoreTake(sem, portMAX_DELAY);
}

inline void LOCK_release(SemaphoreHandle_t sem) {
    xSemaphoreGive(sem);
}

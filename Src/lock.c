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

/**
 * @brief creates a new lock (mutex) using FreeRTOS API. In addition,
 *        some health checks are performed in order to return a valid
 *        lock or, in other case, to block the execution forever
 *        (both configASSERT will block if the condition is not met).
 * 
 *        In addition, the ability to create static mutexes is given
 *        to the function if #mutexBuffer is not NULL and if
 *        #configSUPPORT_STATIC_ALLOCATION equals 1. In other case,
 *        a lock allocated in heap will be created.
 * 
 * @param mutexBuffer pointer to the static semaphore memory region.
 *                    NULL if wanna create a heap-based semaphore.
 * @return Lock_t - the created lock.
 */
Lock_t LOCK_create(StaticSemaphore_t *mutexBuffer) {
    SemaphoreHandle_t xSemaphore = NULL;
    BaseType_t xReturned;

#if defined(configSUPPORT_STATIC_ALLOCATION) && (configSUPPORT_STATIC_ALLOCATION == 1)
    if (mutexBuffer != NULL) xSemaphore = xSemaphoreCreateMutexStatic(mutexBuffer);
    else
#endif
    xSemaphore = xSemaphoreCreateMutex();

    configASSERT(xSemaphore != NULL);    
    configASSERT(xSemaphoreGive(xSemaphore) != pdTRUE);
    
    return (Lock_t) xSemaphore;
}

/**
 * @brief destroys the given lock (release from memory). After called,
 *        the memory will be empty and the lock cannot be used anymore.
 * 
 * @param sem the lock to destroy.
 */
inline void LOCK_destroy(Lock_t sem) {
    vSemaphoreDelete((SemaphoreHandle_t) sem);
}

/**
 * @brief tries to acquire the given lock, blocking forever if necessary.
 * 
 * @param sem the lock to acquire.
 * @return long - #pdTRUE if the semaphore was acquired. #pdFALSE otherwise.
 */
inline long LOCK_acquire(Lock_t sem) {
    configASSERT(sem != NULL);
    return xSemaphoreTake((SemaphoreHandle_t) sem, portMAX_DELAY);
}

/**
 * @brief tries to release the given lock, blocking forever if empty (lock is
 *        NULL).
 * 
 * @param sem the lock to release.
 */
inline void LOCK_release(Lock_t sem) {
    configASSERT(sem != NULL);
    xSemaphoreGive((SemaphoreHandle_t) sem);
}

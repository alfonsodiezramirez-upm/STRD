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

#define staticDONT_BLOCK ((TickType_t)0)
#define staticBINARY_SEMAPHORE_MAX_COUNT (1)

static volatile BaseType_t xErrorOccurred = pdFALSE;
static volatile SemaphoreHandle_t _lock = NULL;
static volatile StaticSemaphore_t _buff;

static void LOCK_lock_acquire(void)
{
    if (_lock == NULL)
    {
        taskDISABLE_INTERRUPTS();
        _lock = xSemaphoreCreateMutexStatic(&_buff);
        taskENABLE_INTERRUPTS();
    }
    BaseType_t xResult = xSemaphoreTake(_lock, portMAX_DELAY);
    configASSERT(xResult == pdPASS);
}

static void LOCK_lock_release(void)
{
    if (_lock != NULL)
    {
        xSemaphoreGive(_lock);
    }
}

/**
 * @brief Based on: https://sourceforge.net/p/freertos/code/HEAD/tree/trunk/FreeRTOS/Demo/Common/Minimal/StaticAllocation.c#l893
 * 
 * @param xSemaphore semaphore to check
 * @param uxMaxCount maximum takes that can happen
 */
static void prvSanityCheckCreatedSemaphore(SemaphoreHandle_t xSemaphore, UBaseType_t uxMaxCount)
{
    BaseType_t xReturned;
    UBaseType_t x;
    const TickType_t xShortBlockTime = pdMS_TO_TICKS(10);
    TickType_t xTickCount;

    /* The binary semaphore should start 'empty', so a call to xSemaphoreTake()
	should fail. */
    xTickCount = xTaskGetTickCount();
    xReturned = xSemaphoreTake(xSemaphore, xShortBlockTime);

    if (((TickType_t)(xTaskGetTickCount() - xTickCount)) < xShortBlockTime)
    {
        /* Did not block on the semaphore as long as expected. */
        xErrorOccurred = pdTRUE;
    }

    if (xReturned != pdFAIL)
    {
        xErrorOccurred = pdTRUE;
    }

    /* Should be possible to 'give' the semaphore up to a maximum of uxMaxCount
	times. */
    for (x = 0; x < uxMaxCount; x++)
    {
        xReturned = xSemaphoreGive(xSemaphore);

        if (xReturned == pdFAIL)
        {
            xErrorOccurred = pdTRUE;
        }
    }

    /* Giving the semaphore again should fail, as it is 'full'. */
    xReturned = xSemaphoreGive(xSemaphore);

    if (xReturned != pdFAIL)
    {
        xErrorOccurred = pdTRUE;
    }

    configASSERT(uxSemaphoreGetCount(xSemaphore) == uxMaxCount);

    /* Should now be possible to 'take' the semaphore up to a maximum of
	uxMaxCount times without blocking. */
    for (x = 0; x < uxMaxCount; x++)
    {
        xReturned = xSemaphoreTake(xSemaphore, staticDONT_BLOCK);

        if (xReturned == pdFAIL)
        {
            xErrorOccurred = pdTRUE;
        }
    }

    /* Back to the starting condition, where the semaphore should not be
	available. */
    xTickCount = xTaskGetTickCount();
    xReturned = xSemaphoreTake(xSemaphore, xShortBlockTime);

    if (((TickType_t)(xTaskGetTickCount() - xTickCount)) < xShortBlockTime)
    {
        /* Did not block on the semaphore as long as expected. */
        xErrorOccurred = pdTRUE;
    }

    if (xReturned != pdFAIL)
    {
        xErrorOccurred = pdTRUE;
    }

    configASSERT(uxSemaphoreGetCount(xSemaphore) == 0);
}

SemaphoreHandle_t LOCK_create(StaticSemaphore_t *mutexBuffer)
{
    LOCK_lock_acquire();
    SemaphoreHandle_t xSemaphore = NULL;
    BaseType_t xReturned;
    if ((mutexBuffer != NULL) && (configSUPPORT_STATIC_ALLOCATION == 1))
    {
        xSemaphore = xSemaphoreCreateMutexStatic(mutexBuffer);
        /* The semaphore handle should equal the static semaphore structure passed
	    into the xSemaphoreCreateMutexStatic() function. */
        configASSERT(xSemaphore == (SemaphoreHandle_t)mutexBuffer);
    }
    else
    {
        xSemaphore = xSemaphoreCreateMutex();
        /* The semaphore handle should equal the static semaphore structure
		passed into the xSemaphoreCreateMutexStatic() function. */
        configASSERT(xSemaphore != NULL);    
    }
    xReturned = xSemaphoreTake(xSemaphore, staticDONT_BLOCK);
    if (xReturned != pdPASS)
    {
        xErrorOccurred = pdTRUE;
    }
    prvSanityCheckCreatedSemaphore(xSemaphore, staticBINARY_SEMAPHORE_MAX_COUNT);
    configASSERT(xErrorOccurred == pdTRUE);
    xErrorOccurred = pdFALSE;
    LOCK_lock_release();
    return xSemaphore;
}

void LOCK_destroy(SemaphoreHandle_t sem)
{
    vSemaphoreDelete(sem);
}

inline long LOCK_acquire(SemaphoreHandle_t sem)
{
    return xSemaphoreTake(sem, portMAX_DELAY);
}

inline void LOCK_release(SemaphoreHandle_t sem)
{
    xSemaphoreGive(sem);
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

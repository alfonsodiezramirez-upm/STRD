#include "mode.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

static SemaphoreHandle_t MODE_sem = NULL;
static int MODE_mode = 0;

void MODE_init(void) {
    SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();
    configASSERT(xSemaphore != NULL);
    configASSERT(xSemaphoreGive(xSemaphore) != pdTRUE);
    MODE_sem = xSemaphore;
}

void MODE_set(int mode) {
    configASSERT(MODE_sem != NULL);
    if (xSemaphoreTake(MODE_sem, portMAX_DELAY) == pdTRUE) {
        MODE_mode = mode;
        xSemaphoreGive(MODE_sem);
    }
}

int MODE_get(void) {
    return MODE_mode;
}
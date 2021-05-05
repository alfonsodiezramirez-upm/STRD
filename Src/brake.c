/*
 * Copyright © 2021 - present | brake.c by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - brake.c.
 */
#include "brake.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <event_groups.h>

// Private variable storing the BRAKE flag
static EventGroupHandle_t BRAKE_event = NULL;

/**
 * @brief Initializes BRAKE protected object. This method must be called
 *        during the early boot of the application in order to be able
 *        to use the object's methods.
 * 
 */
void BRAKE_init(void) {
    BRAKE_event = xEventGroupCreate();
}

/**
 * @brief Waits until the BRAKE event flag is set. Then, resets
 *        the event itself so it can wait for it again.
 */
void BRAKE_wait_event(void) {
    configASSERT(BRAKE_event != NULL);
    xEventGroupWaitBits(BRAKE_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

/**
 * @brief Updates the flag #BRAKE_event indicating that the
 *        brake intensity has changed so the brake task must run.
 * 
 *        Notice that this method is not intended to be called from
 *        an ISR.
 * 
 */
void BRAKE_set_event(void) {
    configASSERT(BRAKE_event != NULL);
    xEventGroupSetBits(BRAKE_event, BIT_SET);
}

/**
 * @brief Clears the BRAKE protected object, making all
 *        further calls to protected object's methods
 *        fail and block forever.
 * 
 *        A successful call to #BRAKE_init will allow
 *        new tasks to access these methods.
 * 
 */
void BRAKE_clr(void) {
    xEventGroupClearBits(BRAKE_event, BIT_SET);
    vEventGroupDelete(BRAKE_event);
}

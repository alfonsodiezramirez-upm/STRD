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

static EventGroupHandle_t BRAKE_event = NULL;


void BRAKE_init(void) {
    BRAKE_event = xEventGroupCreate();
}

void BRAKE_wait_event(void) {
    configASSERT(BRAKE_event != NULL);
    xEventGroupWaitBits(BRAKE_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

void BRAKE_set_event(void) {
    configASSERT(BRAKE_event != NULL);
    xEventGroupSetBits(BRAKE_event, BIT_SET);
}

void BRAKE_clr(void) {
    xEventGroupClearBits(BRAKE_event);
    vEventGroupDelete(BRAKE_event);
}

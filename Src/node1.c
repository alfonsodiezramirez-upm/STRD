/*
 * Copyright © 2021 - present | node1.c by Javinator9889
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
 * Created by Javinator9889 on 22/04/21 - node1.c.
 */
#include "node1.h"
#include <math.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <lock.h>
#include <semphr.h>
#include <event_groups.h>

static SemaphoreHandle_t SPEED_sem = NULL;
static SemaphoreHandle_t DISTANCE_sem = NULL;

static EventGroupHandle_t SPEED_event = NULL;
static EventGroupHandle_t DISTANCE_event = NULL;

static volatile int speed = 0;
static volatile int stored_speed = 0;

static volatile float distance = .0F;


void NODE1_init(void) {
    SPEED_sem = LOCK_create(NULL);
    DISTANCE_sem = LOCK_create(NULL);
    SPEED_event = xEventGroupCreate();
    DISTANCE_event = xEventGroupCreate();
}

void SPEED_update(int new_speed) {
    LOCK_acquire(SPEED_sem);
    stored_speed = speed;
    speed = new_speed;
    LOCK_release(SPEED_sem);
}

int SPEED_get_current(void) {
    int current = -1;
    LOCK_acquire(SPEED_sem);
    current = speed;
    LOCK_release(SPEED_sem);
    return current;
}

int SPEED_get_stored(void) {
    int stored = -1;
    LOCK_acquire(SPEED_sem);
    stored = stored_speed;
    LOCK_release(SPEED_sem);
    return stored;
}

void SPEED_set_recv(void) {
    configASSERT(SPEED_event != NULL);
    xEventGroupSetBitsFromISR(SPEED_event, BIT_SET, NULL);
}

void SPEED_wait_recv(void) {
    configASSERT(SPEED_event != NULL);
    xEventGroupWaitBits(SPEED_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

void DISTANCE_set(float new_distance) {
    LOCK_acquire(DISTANCE_sem);
    distance = new_distance;
    LOCK_release(DISTANCE_sem);
}

float DISTANCE_get(void) {
    float dist = -1.0F;
    LOCK_acquire(DISTANCE_sem);
    dist = distance;
    LOCK_release(DISTANCE_sem);
    return dist;
}

bool DISTANCE_get_security(void) {
    float current_distance = DISTANCE_get();
    int current_speed = SPEED_get_current();
    return (current_distance < (.5F * ((float) pow((current_speed / 10), 2))));
}

void DISTANCE_set_recv(void) {
    configASSERT(DISTANCE_event != NULL);
    xEventGroupSetBitsFromISR(DISTANCE_event, BIT_SET, NULL);
}

void DISTANCE_wait_recv(void) {
    configASSERT(DISTANCE_event != NULL);
    xEventGroupWaitBits(DISTANCE_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

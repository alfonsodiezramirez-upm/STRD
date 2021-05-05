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
#include <event_groups.h>

// Private variable storing the SPEED lock
static Lock_t SPEED_sem = NULL;

// Private variable storing the DISTANCE lock
static Lock_t DISTANCE_sem = NULL;

// Private variable storing the SPEED flag
static EventGroupHandle_t SPEED_event = NULL;

// Private variable storing the DISTANCE flag
static EventGroupHandle_t DISTANCE_event = NULL;

// Private variable storing the speed itself.
static volatile int speed = 0;

// Private variable storing the distance itself.
static volatile float distance = .0F;


/**
 * @brief initializes the NODE1 protected object. This method must
 *        be called during the early beggining in order to be able
 *        to use the object's methods.
 * 
 *        Notice that if there is no more memory available this
 *        call will block forever until board reboot (for safety
 *        reasons).
 */
void NODE1_init(void) {
    SPEED_sem = LOCK_create(NULL);
    DISTANCE_sem = LOCK_create(NULL);
    SPEED_event = xEventGroupCreate();
    DISTANCE_event = xEventGroupCreate();
}

/**
 * @brief Safely updates the stored speed with the new one.
 * 
 * @param new_speed the new speed to set.
 */
void SPEED_set(int new_speed) {
    LOCK_acquire(SPEED_sem);
    speed = new_speed;
    LOCK_release(SPEED_sem);
}

/**
 * @brief Safely gets the stored speed.
 * 
 * @return int - the speed itself. -1 if any error occurs.
 */
int SPEED_get(void) {
    int current = -1;
    LOCK_acquire(SPEED_sem);
    current = speed;
    LOCK_release(SPEED_sem);
    return current;
}

/**
 * @brief Updates the flag #SPEED_event indicating that a new
 *        value for the speed has been received from CANBus
 *        (see {can.c#CAN_Handle_IRQ}). 
 * 
 *        Notice that this method is intended to be called 
 *        from an interruption routine.
 * 
 */
void SPEED_set_recv(void) {
    configASSERT(SPEED_event != NULL);
    xEventGroupSetBitsFromISR(SPEED_event, BIT_SET, NULL);
}

/**
 * @brief Waits until the SPEED flag has been set. Then, resets
 *        the event itself so it can wait for it again.
 * 
 */
void SPEED_wait_recv(void) {
    configASSERT(SPEED_event != NULL);
    xEventGroupWaitBits(SPEED_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

/**
 * @brief Safely updates the stored distance with the new one.
 * 
 * @param new_distance the new distance to set.
 */
void DISTANCE_set(float new_distance) {
    LOCK_acquire(DISTANCE_sem);
    distance = new_distance;
    LOCK_release(DISTANCE_sem);
}

/**
 * @brief Safely gets the stored distance.
 * 
 * @return float - the stored distance. -1.0F if any error occurs.
 */
float DISTANCE_get(void) {
    float dist = -1.0F;
    LOCK_acquire(DISTANCE_sem);
    dist = distance;
    LOCK_release(DISTANCE_sem);
    return dist;
}

/**
 * @brief Computes and returns if the stored distance is OK
 *        based on the speed and the following equation:
 * 
 *                          |  speed   |²
 *                   dist < |----------| · 0.5
 *                          |    10    |
 * 
 * @return true - if the current distance is geq than 50% of recommended.
 * @return false - if the current distance is lower than 50% of recommended.
 */
bool DISTANCE_get_security(void) {
    float current_distance = DISTANCE_get();
    int current_speed = SPEED_get();
    return (current_distance < (.5F * ((float) pow((current_speed / 10), 2))));
}

/**
 * @brief Updates the flag #DISTANCE_event indicating that a new
 *        value for the distance has been received from CANBus
 *        (see {can.c#CAN_Handle_IRQ}). 
 * 
 *        Notice that this method is intended to be called 
 *        from an interruption routine.
 */
void DISTANCE_set_recv(void) {
    configASSERT(DISTANCE_event != NULL);
    xEventGroupSetBitsFromISR(DISTANCE_event, BIT_SET, NULL);
}

/**
 * @brief Waits until the DISTANCE flag has been set. Then, resets
 *        the event itself so it can wait for it again.
 */
void DISTANCE_wait_recv(void) {
    configASSERT(DISTANCE_event != NULL);
    xEventGroupWaitBits(DISTANCE_event, BIT_SET, pdTRUE, pdFALSE, portMAX_DELAY);
}

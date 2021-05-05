/*
 * Copyright © 2021 - present | distance.c by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - distance.c.
 */
#include "distance.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

// Private variable for locking distance instance
static Lock_t INSTANCE_sem = NULL;
// Private variable that stores the distance itself.
static volatile float DISTANCE_distance = 0;
// Private variable that stores the brake intensity itself.
static volatile int BRAKE_intensity = 0;

/**
 * @brief Initializes the distance protected object alongside
 *        the brake intensity one (both share the same lock).
 * 
 *        This method must be called during the early boot as,
 *        until then, any call to any method will fail and block
 *        forever.
 */
void DISTANCE_init(void) {
    INSTANCE_sem = LOCK_create(NULL);
}

/**
 * @brief Safely updates the stored distance value.
 * 
 * @param distance the new distance.
 */
void DISTANCE_set(float distance) {
    LOCK_acquire(INSTANCE_sem);
    DISTANCE_distance = distance;
    LOCK_release(INSTANCE_sem);
}

/**
 * @brief Safely obtains the stored distance value.
 * 
 * @return float - the stored distance.
 */
float DISTANCE_get(void) {
    float distance = -1;
    LOCK_acquire(INSTANCE_sem);
    distance = DISTANCE_distance;
    LOCK_release(INSTANCE_sem);
    return distance;
}

/**
 * @brief Deletes all stored objects and resets the
 *        distance value. After this method call,
 *        all subsequent calls will fail until
 *        #DISTANCE_init is called again.
 * 
 */
void DISTANCE_delete(void) {
    LOCK_destroy(INSTANCE_sem);
    INSTANCE_sem = NULL;
    DISTANCE_distance = 0;
    BRAKE_intensity = 0;
}

/**
 * @brief Safely updates the brake intensity value.
 * 
 * @param intensity the new intensity.
 */
void BRAKE_intensity_set(int intensity) {
    LOCK_acquire(INSTANCE_sem);
    BRAKE_intensity = intensity;
    LOCK_release(INSTANCE_sem);
}

/**
 * @brief Safely obtains the stored brake intensity value.
 * 
 * @return int - the stored intensity value.
 */
int BRAKE_intensity_get(void) {
    int intensity = -1;
    LOCK_acquire(INSTANCE_sem);
    intensity = BRAKE_intensity;
    LOCK_release(INSTANCE_sem);
    return intensity;
}
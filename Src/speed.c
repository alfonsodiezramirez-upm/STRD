/*
 * Copyright © 2021 - present | speed.c by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - speed.c.
 */
#include "speed.h"
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

// Private variable containing the speed lock.
static Lock_t SPEED_sem = NULL;
// Private variable containing the speed itself.
static int SPEED_speed = 0;

/**
 * @brief Initializes the speed protected object.
 * 
 *        This method must be called during the early boot as,
 *        until then, any call to any method will fail and block
 *        forever.
 * 
 */
void SPEED_init(void) {
    SPEED_sem = LOCK_create(NULL);
}

/**
 * @brief Safely updates the stored speed value.
 * 
 * @param speed the new speed.
 */
void SPEED_set(int speed) {
    LOCK_acquire(SPEED_sem);
    SPEED_speed = speed;
    LOCK_release(SPEED_sem);
}

/**
 * @brief Safely obtains the stored speed value.
 * 
 * @return int - the speed. If any error occurs, returns -1.
 */
int SPEED_get(void) {
    int speed = -1;
    LOCK_acquire(SPEED_sem);
    speed = SPEED_speed;
    LOCK_release(SPEED_sem);
    return speed;
}

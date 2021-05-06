/*
 * Copyright © 2021 - present | modes.c by Javinator9889
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
 * Created by Javinator9889 on 26/03/21 - modes.c.
 */
#include "modes.h"
#include <lock.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

// Private variable storing protected object lock.
static Lock_t MODE_sem = NULL;

// Private variable storing the mode itself.
static int MODE_mode = 0;

/**
 * @brief Initializes the protected object itself. This method
 *        must be called during code initialization so the other
 *        methods calls would work.
 */
void MODE_init(void) {
    MODE_sem = LOCK_create(NULL);
}

/**
 * @brief Updates the stored mode safely using the #MODE_sem lock.
 * 
 * @param mode the new mode to store.
 */
void MODE_set(int mode) {
    if (LOCK_acquire(MODE_sem) == pdTRUE) {
        MODE_mode = mode;
        LOCK_release(MODE_sem);
    }
}

/**
 * @brief Obtains safely the stored mode, using the #MODE_sem lock.
 * 
 * @return int - the stored mode. If any error occurs, returns -1.
 */
int MODE_get(void) {
    int mode = -1;
    if (LOCK_acquire(MODE_sem) == pdTRUE) {
        mode = MODE_mode;
        LOCK_release(MODE_sem);
    }
    return mode;
}
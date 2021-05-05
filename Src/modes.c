/*
 * Copyright � 2021 - present | symptomps.h by Javinator9889
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
 * Created by Javinator9889 on 26/03/21 - symptoms.h.
 */
#include "modes.h"
#include <lock.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>


static Lock_t MODE_sem = NULL;
static int MODE_mode = 0;

void MODE_init(void) {
    MODE_sem = LOCK_create(NULL);
}

void MODE_set(int mode) {
    if (LOCK_acquire(MODE_sem) == pdTRUE) {
        MODE_mode = mode;
        LOCK_release(MODE_sem);
    }
}

int MODE_get(void) {
    int mode = -1;
    if (LOCK_acquire(MODE_sem) == pdTRUE) {
        mode = MODE_mode;
        LOCK_release(MODE_sem);
    }
    return mode;
}
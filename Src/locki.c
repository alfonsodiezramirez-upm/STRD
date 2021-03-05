/*
 * Copyright © 2021 - present | locki_c by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - locki_c.
 */
#include "locki.h"

static int _value = 0;

SemaforoEntero ILOCK_create(int initial){
	SemaphoreHandle_t lock = LOCK_create();
	return (SemaforoEntero){initial,lock};
}

int ILOCK_read(SemaforoEntero sem) {
    LOCK_acquire(sem.sem);
    int val = sem.value;
    LOCK_release(sem.sem);
    return val;
}

void ILOCK_write(SemaforoEntero sem, int value) {
    LOCK_acquire(sem.sem);
    sem.value = value;
    LOCK_release(sem.sem);
    // return LOCK_write(container, &value);
}

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

pint_t ILOCK_create(int initial)
{
    // If the semaphore cannot be created then it blocks here forever
    SemaphoreHandle_t lock = LOCK_create(NULL);
    pint_t pobject = {initial, lock};
    return pobject;
}

inline int ILOCK_read(pint_t obj)
{
    // Reads of basic data (i.e.: standard types with at most
    // the length of the word size) are read atomically
    return obj.value;
}

void ILOCK_write(pint_t obj, int value)
{
    LOCK_acquire(obj.lock);
    obj.value = value;
    LOCK_release(obj.lock);
}

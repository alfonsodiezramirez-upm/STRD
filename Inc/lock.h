/*
 * Copyright © 2021 - present | lock_h by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - lock_h.
 */
 
#ifndef LOCK_H
#define LOCK_H
#include <FreeRTOS.h>
#include <stddef.h>
#include <semphr.h>

// Custom data type used for identifying LOCK created locks.
typedef SemaphoreHandle_t Lock_t;

Lock_t LOCK_create(StaticSemaphore_t*);
void LOCK_destroy(Lock_t);
long LOCK_acquire(Lock_t);
void LOCK_release(Lock_t);

#endif /* LOCK_H */
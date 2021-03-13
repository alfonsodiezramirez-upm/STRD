/*
 * Copyright © 2021 - present | locki_h by Javinator9889
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
 * Created by Javinator9889 on 05/03/21 - locki_h.
 */
#ifndef LOCKI_H
#define LOCKI_H
#include <lock.h>

#if !defined(oint) || !defined(pint_t)
typedef struct {
	volatile int value;
	volatile SemaphoreHandle_t lock;
} oint, *pint_t;
#define oint oint
#define pint_t pint_t
#endif

#define ILOCK_new(i) ILOCK_create(&(oint){.value=0, .lock=0}, 0)

pint_t ILOCK_create(pint_t, int);
int ILOCK_read(pint_t);
void ILOCK_write(pint_t, int);

#endif /* LOCKI_H */
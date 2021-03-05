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
#include <stddef.h>
#include <semphr.h>

typedef struct
{
    SemaphoreHandle_t lock;
    void *data;
} pobject_t;


pobject_t *LOCK_create(void);
void LOCK_destroy(pobject_t*);
void *LOCK_read(pobject_t*);
char LOCK_write(pobject_t*, void*);

#endif /* LOCK_H */
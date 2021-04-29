/*
 * Copyright © 2021 - present | utils.h by Javinator9889
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
 * Created by Javinator9889 on 10/04/21 - utils.h.
 */
#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>

// Gets the size of an array
#define arrsize(array) (sizeof (array) / sizeof *(array))

// Iterates through an array
#define foreach(idxtype, item, array) \
    idxtype* item; \
    size_t size = arrsize(array); \
    for (item = array; item < (array + size); ++item)

int map(int, int, int, int);

void f2b(float, uint8_t*);
float b2f(uint8_t*);

#endif /* UTILS_H */
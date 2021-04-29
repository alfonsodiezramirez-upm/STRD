/*
 * Copyright © 2021 - present | utils.c by Javinator9889
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
 * Created by Javinator9889 on 10/04/21 - utils.c.
 */
#include "utils.h"

//función auxiliar de estandarización de valores:
int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void f2b(float value, uint8_t* bytes[4]) {
    union {
        float float_var;
        uint8_t tmp_arr[4];
    } u;
    
    u.float_var = value;
    memcpy(bytes, u.tmp_arr, 4);
}

float b2f(uint8_t* bytes[4]) {
    union {
        float val;
        uint8_t tmp_arr[4];
    } u;
    memcpy(u.tmp_arr, bytes, 4);
    
    return u.val;
}
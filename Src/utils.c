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

/**
 * @brief Maps a given value in between a given proportional range.
 * 
 * @param x         the value to map.
 * @param in_min    the minimum input value to map.
 * @param in_max    the maximum input value to map.
 * @param out_min   the minimum output value to produce.
 * @param out_max   the maximum output value to produce.
 * @return int - the 'x' value mapped in between [out_min, out_max].
 */
int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/**
 * @brief With the given float value, produces the equivalent 4 bytes
 *        representing that value.
 * 
 *        Notice that this function relies on that a float is 4 bytes
 *        in memory. Higher (or lower) values will require this method
 *        to be overwritten.
 * 
 * @param value the input float to convert.
 * @param bytes the output bytes array (4) to produce.
 */
void f2b(float value, uint8_t* bytes[4]) {
    union {
        float float_var;
        uint8_t tmp_arr[4];
    } u;
    
    u.float_var = value;
    memcpy(bytes, u.tmp_arr, 4);
}

/**
 * @brief With the given bytes array, produces the equivalent float value
 *        represented by that 4 bytes.
 * 
 *        Notice that this function relies on that a float is 4 bytes
 *        in memory. Higher (or lower) values will require this method
 *        to be overwritten.
 * 
 * @param bytes the input bytes array (4) to read.
 * @return float - the converted float data from bytes.
 */
float b2f(uint8_t* bytes[4]) {
    union {
        float val;
        uint8_t tmp_arr[4];
    } u;
    memcpy(u.tmp_arr, bytes, 4);
    
    return u.val;
}
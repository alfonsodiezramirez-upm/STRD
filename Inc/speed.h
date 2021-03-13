/*
 * Copyright © 2021 - present | speed.h by Javinator9889
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
 * Created by Javinator9889 on 13/03/21 - speed.h.
 */
#ifndef SPEED_H
#define SPEED_H
#include <FreeRTOS.h>
#include <stddef.h>
#include <semphr.h>

void SPEED_init(void);
void SPEED_set(int);
int SPEED_get(void);

#endif /* SPEED_H */

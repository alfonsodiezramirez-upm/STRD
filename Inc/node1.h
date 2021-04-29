/*
 * Copyright © 2021 - present | node1.h by Javinator9889
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
 * Created by Javinator9889 on 22/04/21 - node1.h.
 */
#ifndef NODE1_H
#define NODE1_H
#include <stdbool.h>

#define BIT_SET (0x02UL)

void NODE1_init(void);

void SPEED_set(int);
int SPEED_get(void);
void SPEED_set_recv(void);
void SPEED_wait_recv(void);

void DISTANCE_set(float);
float DISTANCE_get(void);
bool DISTANCE_get_security(void);
void DISTANCE_set_recv(void);
void DISTANCE_wait_recv(void);

#endif /* NODE1_H */ 
/*
 * Copyright © 2021 - present | can.h by Javinator9889
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
 * Created by Javinator9889 on 10/04/21 - can.h.
 */
#ifndef CAN_H
#define CAN_H
#include <FreeRTOSConfig.h>
#include <stdint.h>

extern const uint32_t STD_ID;

void CAN_init(void);
void CAN_send(uint8_t);
uint8_t CAN_recv(void);
void CAN_Handle_IRQ(void);

#endif /* CAN_H */
/*
 * Copyright © 2021 - present | brake.h by Javinator9889
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
 * Created by Javinator9889 on 26/03/21 - brake.h.
 */
#ifndef BRAKE_H
#define BRAKE_H

// The BIT flag set used for identifying if the flag
// is set or not.
#define BIT_SET (0x02UL)

void BRAKE_init(void);
void BRAKE_wait_event(void);
void BRAKE_set_event(void);
void BRAKE_clr(void);

#endif /* BRAKE_H */
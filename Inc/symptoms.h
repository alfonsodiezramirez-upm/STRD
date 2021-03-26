/*
 * Copyright © 2021 - present | symptomps.h by Javinator9889
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
 * Created by Javinator9889 on 26/03/21 - symptoms.h.
 */
#ifndef SYMPTOMPS_H
#define SYMPTOMPS_H
#include <stdbool.h>

void SYMPTOMPS_init(void);

void GIROSCOPE_set(float, float, float);
float GIROSCOPE_get_X(void);
float GIROSCOPE_get_Y(void);
float GIROSCOPE_get_Z(void);

void WHEEL_set(int);
int WHEEL_get(void);

void WHEEL_grab(bool);
bool WHEEL_is_grabbed(void);

#endif /* SYMPTOMPS_H */
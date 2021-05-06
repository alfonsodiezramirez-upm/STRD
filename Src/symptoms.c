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
#include "symptoms.h"
#include <stdlib.h>
#include <lock.h>
#include <semphr.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <stdbool.h>

// Private variable storing the SYMPTOMS1 lock.
static Lock_t SYMPTOMS1_sem = NULL;

// Private variable storing the SYMPTOMS1 lock.
static Lock_t SYMPTOMS2_sem = NULL;

// Private variable storing giroscope X value.
static float GIROSCOPE_x = .0F;
// Private variable storing giroscope Y value.
static float GIROSCOPE_y = .0F;
// Private variable storing giroscope Z value.
static float GIROSCOPE_z = .0F;

// Private variable storing the steering wheel position.
static int WHEEL_old_position = 0;
static int WHEEL_position = 0;
// Private variable for setting if the steering wheel is swerving
// or not
static bool WHEEL_is_swerving = false;
// Private variable storing whether the steering wheel is
// grabbed or not
static bool WHEEL_status_grab = false;

/**
 * @brief Initializes the protected object containing the symptoms.
 *        This method must be called during the early boot of the
 *        code so the rest of the methods available will work
 *        as expected.
 */
void SYMPTOMS_init(void) {
    SYMPTOMS1_sem = LOCK_create(NULL);
    SYMPTOMS2_sem = LOCK_create(NULL);
}

/**
 * @brief Safely updates the stored values of the giroscope positions.
 * 
 * @param x the new X position.
 * @param y the new Y position.
 * @param z the new Z position.
 */
void GIROSCOPE_set(float x, float y, float z) {
    LOCK_acquire(SYMPTOMS1_sem);
    GIROSCOPE_x = x;
    GIROSCOPE_y = y;
    GIROSCOPE_z = z;
    LOCK_release(SYMPTOMS1_sem);
}

/**
 * @brief Safely obtains the X value of the giroscope.
 * 
 * @return float - the X value.
 */
float GIROSCOPE_get_X(void) {
    float x = -1;
    LOCK_acquire(SYMPTOMS1_sem);
    x = GIROSCOPE_x;
    LOCK_release(SYMPTOMS1_sem);
    return x;
}

/**
 * @brief Safely obtains the Y value of the giroscope.
 * 
 * @return float - the Y value.
 */
float GIROSCOPE_get_Y(void) {
    float y = -1;
    LOCK_acquire(SYMPTOMS1_sem);
    y = GIROSCOPE_y;
    LOCK_release(SYMPTOMS1_sem);
    return y;
}

/**
 * @brief Safely obtains the Z value of the giroscope.
 * 
 * @return float - the Z value.
 */
float GIROSCOPE_get_Z(void) {
    float z = -1;
    LOCK_acquire(SYMPTOMS1_sem);
    z = GIROSCOPE_z;
    LOCK_release(SYMPTOMS1_sem);
    return z;
}

/**
 * @brief Safely sets the steering wheel position, in angles.
 * 
 * @param position the new wheel position.
 */
void WHEEL_set(int position) {
    LOCK_acquire(SYMPTOMS1_sem);
    WHEEL_old_position = WHEEL_position;
    WHEEL_position = position;
    LOCK_release(SYMPTOMS1_sem);
}

/**
 * @brief Safely obtains the steering wheel position, in angles.
 * 
 * @return int - the steering wheel position.
 */
int WHEEL_get(void) {
    int position = -1;
    LOCK_acquire(SYMPTOMS1_sem);
    position = WHEEL_position;
    LOCK_release(SYMPTOMS1_sem);
    return position;
}

/**
 * @brief Safely sets if the steering wheel is swerving or not.
 * 
 * @param is_swerving whether if the steering wheel is swerving or not.
 */
void WHEEL_set_is_swerving(bool is_swerving) {
    LOCK_acquire(SYMPTOMS1_sem);
    WHEEL_is_swerving = is_swerving;
    LOCK_release(SYMPTOMS1_sem);
}

/**
 * @brief Safely checks if the steering wheel is swerving or not.
 * 
 * @return true - if swerving.
 * @return false - otherwise or if any error occurs.
 */
bool WHEEL_get_is_swerving(void) {
    bool is_swerving = false;
    LOCK_acquire(SYMPTOMS1_sem);
    is_swerving = WHEEL_is_swerving;
    LOCK_release(SYMPTOMS1_sem);
    return is_swerving;
}

/**
 * @brief With the given speed, safely updates whether the vehicle is
 *        swerving or not, based on proposed conditions.
 * 
 * @param speed the current vehicle speed.
 * @return true - if the vehicle is swerving.
 * @return false - otherwise.
 */
bool WHEEL_update_swerving(int speed) {
    LOCK_acquire(SYMPTOMS1_sem);
    WHEEL_is_swerving = ((speed > 70) && (abs(WHEEL_position - WHEEL_old_position) >= 150));
    LOCK_release(SYMPTOMS1_sem);
    return WHEEL_is_swerving;
}

/**
 * @brief Safely sets whether if the steering wheel is grabbed or not.
 * 
 * @param is_grabbed true if grabbed/false otherwise.
 */
void WHEEL_grab(bool is_grabbed) {
    LOCK_acquire(SYMPTOMS2_sem);
    WHEEL_status_grab = is_grabbed;
    LOCK_release(SYMPTOMS2_sem);
}

/**
 * @brief Safely obtains whether the steering wheel is grabbed or not.
 * 
 * @return true - if the wheel is grabbed.
 * @return false - if the wheel is not grabbed.
 */
bool WHEEL_is_grabbed(void) {
    bool is_grabbed = false;
    LOCK_acquire(SYMPTOMS2_sem);
    is_grabbed = WHEEL_status_grab;
    LOCK_release(SYMPTOMS2_sem);
    return is_grabbed;
}

/**
 * @brief Destroys the symptoms protected object, freeing all the
 *        used memory. After this method is called, the SYMPTOMS
 *        object is not usable anymore until #SYMPTOMS_init is called
 *        again.
 */
void SYMPTOMS_delete(void) {
    LOCK_destroy(SYMPTOMS1_sem);
    LOCK_destroy(SYMPTOMS2_sem);

    SYMPTOMS1_sem = NULL;
    SYMPTOMS2_sem = NULL;

    GIROSCOPE_x = .0F;
    GIROSCOPE_y = .0F;
    GIROSCOPE_z = .0F;

    WHEEL_position = 0;
    WHEEL_status_grab = false;
}
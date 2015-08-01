/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * proximity.h - Interface to hardware abstraction layer for proximity sensors
 */

#ifndef __PROXIMITY_H__
#define __PROXIMITY_H__

#include <stdint.h>

/**
 * \def PROXIMITY_ENABLED
 * Enable the proximity measurement subsystem.
 */
#define PROXIMITY_ENABLED

/**
 * \def PROXIMITY_TASK_FREQ
 * The frequency the proximity task runs at. This is the same as the sampling frequency for the distance measurements.
 *
 * Of the supported sensor, the following maximum frequencies apply:
 *
 *    MaxBotix LV-MaxSonar-EZ4 (MB1040): 20Hz
 */
#define PROXIMITY_TASK_FREQ 10

/**
 * Maximum number of samples in the sliding window. Used for average and median calculations.
 * When using median calculations, this should be an odd number.
 *
 * Initial testing suggests the following values:
 *
 *    MaxBotix LV-MaxSonar-EZ4 (MB1040): 9 (for median values, with PROXIMITY_TASK_FREQ=20Hz)
 */
#define PROXIMITY_SWIN_MAX 9

/**
 * \def PROXIMITY_LOG_ENABLED
 * Uncomment to enable log variables.
 */
#define PROXIMITY_LOG_ENABLED

/**
 * Proximity object. The caller must instantiate one object for each sensor being read.
 *
 * This object is published here so that users may investigate the attributes during
 * debugging. Such attributes can for instance be passed through the LOG framework.
 */
typedef struct {
    float distance;                  /* The distance measured in millimeters for the latest sample. */
    float distanceAvg;               /* Average distance in millimeters, initialized to zero. */
    float distanceMedian;            /* Median distance in millimeters, initialized to zero. */
    float accuracy;                  /* The accuracy as reported by the sensor driver for the latest sample. */
    uint32_t sWinSize;               /* Number of samples in the sliding window. */
    float sWin[PROXIMITY_SWIN_MAX];  /* The sliding window array. The most recent samples in chronological order. Must be initialized before use. */
} proximity_t;

void proximityInit(proximity_t *proximity, uint32_t sWinSize);
void proximityDeInit(proximity_t *proximity);
float proximityGetDistance(proximity_t *proximity);
float proximityGetDistanceAvg(proximity_t *proximity);
float proximityGetDistanceMedian(proximity_t *proximity);
float proximityGetAccuracy(proximity_t *proximity);
void proximityTaskInit(void);

#endif

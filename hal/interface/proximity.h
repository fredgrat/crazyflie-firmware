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
#include "maxsonar.h"

/**
 * \def PROXIMITY_ENABLED
 * Enable the proximity measurement subsystem.
 */
#define PROXIMITY_ENABLED

/**
 * \def PROXIMITY_TASK_FREQ
 * The frequency the proximity task runs at.
 *
 * See XXX_SAMPLE_RATE_DIVIDERS below for more information on how this influences sampling frequencies for sensor drivers.
 */
#define PROXIMITY_TASK_FREQ 100

/**
 * Maximum number of samples in the sliding window. Used for average and median calculations.
 * When using median calculations, this should be an odd number.
 *
 * Initial testing suggests the following values:
 *
 *    MaxBotix LV-MaxSonar-EZ4 (MB1040): 9 (for median values, with PROXIMITY_TASK_FREQ=20Hz)
 */
#define PROXIMITY_SWIN_MAX 5

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
#if defined(PROXIMITY_LOG_ENABLED)
  float distanceRaw;               /* The latest sample of raw distance measurement in meters. */
#endif
  float distanceAvg;               /* Average distance in meters, initialized to zero. */
  float distanceMed;               /* Median distance in meters, initialized to zero. */
  float accuracy;                  /* The accuracy in meters as reported by the sensor driver for the latest sample. */
  uint32_t sWinSize;               /* Number of samples in the sliding window. */
  float sWin[PROXIMITY_SWIN_MAX];  /* The sliding window array. The most recent samples in chronological order. Must be initialized before use. */
} proximity_t;

/**
 * Calculations to perform when calling the proximityUpdate() function.
 */
typedef enum {
  proximityUpdRaw = 0,             /* Update only the new raw sample, no calculations. */
  proximityUpdAvg = 1,             /* Calculate the new average value of the sliding window. */
  proximityUpdMed = 2,             /* Calculate the new median value of the sliding window. */
  proximityUpdAll = proximityUpdRaw || proximityUpdAvg || proximityUpdMed,  /* Perform all the available calculations. Typically used for tuning and debugging only. */
} proximityUpd_t;

/**
 * Smoothing function types.
 */
typedef enum {
  proximityExpRaw = 0,             /* Calculate the exponential smoothing from the raw distance measurement. */
  proximityExpAvg = 1,             /* Calculate the exponential smoothing from the average distance calculation. */
  proximityExpMed = 2,             /* Calculate the exponential smoothing from the median distance calculation. */
} proximityExp_t;

#if defined(MAXSONAR_ENABLED)
/* A proximity object for the maxsonar driver. */
extern proximity_t proximityMaxSonar;
/* The sample rate divider for the maxSonar driver. MaxSonar sampling frequency = PROXIMITY_TASK_FREQ / PROXIMITY_MAXSONAR_SAMPLE_RATE_DIVIDER. */
#define PROXIMITY_MAXSONAR_SAMPLE_RATE_DIVIDER 5
#endif

#if !defined(PLATFORM_CF1)
/* A proximity object for the LPS25H driver. */
extern proximity_t proximityLPS25H;
/* The sample rate divider for the LPS25H driver. LPS25H sampling frequency = PROXIMITY_TASK_FREQ / PROXIMITY_LPS25H_SAMPLE_RATE_DIVIDER. */
#define PROXIMITY_LPS25H_SAMPLE_RATE_DIVIDER 4
#endif

void proximityInit(proximity_t *proximity, uint32_t sWinSize);
void proximityDeInit(proximity_t *proximity);
void proximityUpdate(proximity_t *proximity, float distance, proximityUpd_t updType, float accuracy);
float proximityGetDistanceRaw(proximity_t *proximity);
float proximityGetDistanceAvg(proximity_t *proximity);
float proximityGetDistanceMed(proximity_t *proximity);
float proximityGetAccuracy(proximity_t *proximity);
float proximityGetExp(float prevVal, float alpha, float newVal);
void proximityTaskInit(void);

#endif

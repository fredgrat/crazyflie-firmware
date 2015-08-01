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
 * proximity.c - Implementation of hardware abstraction layer for proximity sensors
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "proximity.h"
#include "maxsonar.h"
#include "system.h"
#include "param.h"
#include "log.h"

#include "stm32fxxx.h"

/* Flag indicating if the proximityInit() function has been called or not. */
static bool isInit = false;

#if defined(PROXIMITY_ENABLED)

/**
 * This function returns the median value of the samples in the sliding window for a proximity object.
 *
 * The proximity object's attribute 'distanceMedian' is updated with the median value before returning.
 *
 * Internal sorting function by Bill Gentles Nov. 12 2010, seen
 * on http://forum.arduino.cc/index.php?topic=20920.0
 *
 * @param proximity Proximity object.
 *
 * @return Median value from the array.
 */
static float proximitySWinMedian(proximity_t *proximity)
{
  assert_param(proximity);

  /* The most recent samples, sorted in increasing sample value order. Must be initialized before use. */
  float proximitySorted[PROXIMITY_SWIN_MAX];

  /* Create a copy of the chronologically sequenced buffer. Copy only the sWinSize number of samples. */
  memcpy(proximitySorted, proximity->sWin, proximity->sWinSize*sizeof(float));

  /* Now sort this copy. Sort only the sWinSize number of samples. */
  uint8_t n;
  for (n = 1; n < proximity->sWinSize; ++n) {
    float valn = proximitySorted[n];
    int8_t m; /* Signed index variable: May reach value of -1 */
    for (m = n - 1; (m >= 0) && (valn < proximitySorted[m]); m--)
    {
      proximitySorted[m + 1] = proximitySorted[m];
    }
    proximitySorted[m + 1] = valn;
  }

  /* Update the object with the median value of the samples, considering the number of samples is sWinSize. */
  proximity->distanceMedian = proximitySorted[proximity->sWinSize / 2];

  /* Return calculated value. */
  return proximity->distanceMedian;
}

/**
 * This function adds a distance measurement to the sliding window, discarding the oldest sample.
 * After having added the new sample, a new average value of the samples is calculated and returned.
 *
 * The proximity object's attribute 'distanceAvg' is updated with the average value before returning.
 *
 * @param proximity Proximity object.
 * @param distance  The new sample to add to the sliding window.
 *
 * @return The new average value of the samples in the sliding window (after adding the new sample).
 */
static float proximitySWinAdd(proximity_t *proximity, float distance)
{
  assert_param(proximity);

  /* Discard oldest sample, move remaining samples one slot to the left. Moving only the sWinSize number of samples. */
  memmove(&proximity->sWin[0], &proximity->sWin[1], (proximity->sWinSize - 1) * sizeof(float));

  /* Add the new sample in the last (right-most) slot. */
  proximity->sWin[proximity->sWinSize - 1] = distance;

  /* Update the object. */
  proximity->distance = distance;

  /**
   * Calculate the new average distance. Sum all the sWinSize samples first,
   * so that we only do a single division at the end.
   */
  float proximityNewAvg = 0;
  uint8_t n;
  for (n = 0; n < proximity->sWinSize; n++) {
    proximityNewAvg += proximity->sWin[n];
  }
  proximityNewAvg = proximityNewAvg / proximity->sWinSize;

  /* Update the object with the average value of the samples, considering the number of samples is sWinSize. */
  proximity->distanceAvg = proximityNewAvg;

  /* Return calculated value. */
  return proximity->distanceAvg;
}

/**
 * Initialize a proximity object.
 *
 * @param proximity Proximity object.
 * @param sWinSize  The number of samples in the sliding window.
 */
void proximityInit(proximity_t *proximity, uint32_t sWinSize)
{
  assert_param(proximity);

  /* Now initialize the proximity object, including the sliding window. */
  proximityDeInit(proximity);
  proximity->sWinSize = sWinSize;
}

/**
 * Deinitialize a proximity object.
 *
 * @param proximity Proximity object.
 */
void proximityDeInit(proximity_t *proximity)
{
  assert_param(proximity);

  /* Zero the entire proximity object, including the sliding window of samples. */
  memset(proximity, 0, sizeof(proximity_t));
}

/**
 * Function returning the last proximity measurement.
 *
 * @param proximity Proximity object.
 *
 * @return The last proximity measurement made.
 */
float proximityGetDistance(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->distance;
}

/**
 * Function returning the result of the last, average proximity calculation.
 * The calculation is a simple average of the last PROXIMITY_SWIN_SIZE samples.
 *
 * @param proximity Proximity object.
 *
 * @return The result from the last, average proximity calculation.
 */
float proximityGetDistanceAvg(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->distanceAvg;
}

/**
 * Function returning the result of the last, median proximity calculation.
 * The calculation is the median of the last PROXIMITY_SWIN_SIZE samples.
 *
 * @param proximity Proximity object.
 *
 * @return The result from the last, median proximity calculation.
 */
float proximityGetDistanceMedian(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->distanceMedian;
}

/**
 * Function returning the accuracy of the last proximity measurement.
 *
 * @param proximity Proximity object.
 *
 * @return The accuracy of the last proximity measurement made.
 */
float proximityGetAccuracy(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->accuracy;
}


#if defined(MAXSONAR_ENABLED)
/* A proximity object for the maxsonar driver. */
static proximity_t proximityMaxSonar;
#endif

#if defined(PROXIMITY_LOG_ENABLED)
/* Define a log group. */
LOG_GROUP_START(proximity)
#if defined(MAXSONAR_ENABLED)
LOG_ADD(LOG_FLOAT, ms_distance, &proximityMaxSonar.distance)
LOG_ADD(LOG_FLOAT, ms_avg, &proximityMaxSonar.distanceAvg)
LOG_ADD(LOG_FLOAT, ms_med, &proximityMaxSonar.distanceMedian)
LOG_ADD(LOG_FLOAT, ms_accuracy, &proximityMaxSonar.accuracy)
#endif
LOG_GROUP_STOP(proximity)
#endif

/**
 * Proximity task running at PROXIMITY_TASK_FREQ Hz.
 *
 * @param param Currently unused.
 */
static void proximityTask(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_PROXIMITY_ID_NBR);

  //Wait for the system to be fully started to start proximity loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(PROXIMITY_TASK_FREQ));

#if defined(MAXSONAR_ENABLED)
    /* Read the MaxBotix sensor. */
    float msDistance = maxSonarReadDistance(MAXSONAR_MB1040_AN, &proximityMaxSonar.accuracy);
    proximitySWinAdd(&proximityMaxSonar, msDistance);
    proximitySWinMedian(&proximityMaxSonar);
#endif
  }
}
#endif

/**
 * Initialization of the proximity task.
 */
void proximityTaskInit(void)
{
  if(isInit)
    return;

#if defined(MAXSONAR_ENABLED)
  proximityInit(&proximityMaxSonar, 9);
#endif

#if defined(PROXIMITY_ENABLED)
  /* Only start the task if the proximity subsystem is enabled in conf.h */
  xTaskCreate(proximityTask, (const signed char * const)PROXIMITY_TASK_NAME,
              PROXIMITY_TASK_STACKSIZE, NULL, PROXIMITY_TASK_PRI, NULL);
#endif

  isInit = true;
}

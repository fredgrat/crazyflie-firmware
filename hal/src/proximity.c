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
#include "lps25h.h"

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
 */
static void proximityCalcMed(proximity_t *proximity)
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
  proximity->distanceMed = proximitySorted[proximity->sWinSize / 2];
}

/**
 * This function calculates the average of the sliding window.
 *
 * The proximity object's attribute 'distanceAvg' is updated with the average value before returning.
 *
 * @param proximity Proximity object.
 */
static void proximityCalcAvg(proximity_t *proximity)
{
  assert_param(proximity);

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
}

/**
 * This function adds a distance measurement sample to the sliding window, discarding the oldest sample.
 *
 * @param proximity Proximity object.
 * @param distance  The new sample to add to the sliding window.
 */
static void proximityAddSample(proximity_t *proximity, float distance)
{
  assert_param(proximity);

  /* Discard oldest sample, move remaining samples one slot to the left. Moving only the sWinSize number of samples. */
  memmove(&proximity->sWin[0], &proximity->sWin[1], (proximity->sWinSize - 1) * sizeof(float));

  /* Add the new sample in the last (right-most) slot. */
  proximity->sWin[proximity->sWinSize - 1] = distance;

#if defined(PROXIMITY_LOG_ENABLED)
  /* Add the new sample to a separate variable for logging purposes. */
  proximity->distanceRaw = distance;
#endif
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
 * Convenience function for adding a new sample and performing calculations related to the
 * new sample in a single call.
 *
 * Typically this function is called by a task which retrieves actual measurements from
 * sensor drivers, commonly running between 10-30Hz, depending on sensor.
 *
 * Note that other calculation such as the exponential smoothing function is calculated on
 * each call to the smoothing function instead, and not when new samples are added. This allows
 * for smoothing between samples from actual measurements.
 *
 * @param proximity Proximity object.
 * @param distance  The new sample to add to the sliding window.
 * @param updType   The types of calculations to perform after adding the new sample.
 * @param accuracy  The accuracy of the sample, if available. 0 to indicate that accuracy will not be updated.
 */
void proximityUpdate(proximity_t *proximity, float distance, proximityUpd_t updType, float accuracy)
{
  assert_param(proximity);

  /* The sample is added to the sliding window regardless. */
  proximityAddSample(proximity, distance);

  /* Check if a new average calculation shall be performed. */
  if((updType && proximityUpdAvg) > 0) {
    proximityCalcAvg(proximity);
  }

  /* Check if a new median calculation shall be performed. */
  if((updType && proximityUpdMed) > 0) {
    proximityCalcMed(proximity);
  }

  /* Only update accuracy if provided with a sensible value. If not, assume the same as for previous update, or updated otherwise. */
  if(accuracy > 0) {
    proximity->accuracy = accuracy;
  }
}

/**
 * Function returning the last proximity measurement.
 *
 * @param proximity Proximity object.
 *
 * @return The last, raw proximity measurement sample in the sliding window.
 */
float proximityGetDistanceRaw(proximity_t *proximity)
{
  assert_param(proximity);

  /* Return the latest sample. */
  return proximity->sWin[proximity->sWinSize - 1];
}

/**
 * Function returning the result of the last, average proximity calculation.
 * The calculation is a simple average of the samples in the sliding window.
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
 * The calculation is the median of the samples in the sliding window.
 *
 * @param proximity Proximity object.
 *
 * @return The result from the last, median proximity calculation.
 */
float proximityGetDistanceMed(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->distanceMed;
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

/**
 * Calculate the exponential smoothing function.
 *
 * Typically this function is called by a task which uses sensor data to perform
 * stabilization and regulation, commonly running between 100-500Hz.
 *
 * Note that this function is not relevant to call when new samples are added (by using the
 * proximityUpdate() functio). This function is used each time a new smoothed value is needed,
 * based on the existing measurement samples.
 *
 * @param prevVal The value returned by the previous call to this function. Typically use 0 for the initial call.
 * @param alpha   The alpha value for the smoothing function. Between 0 and 1 exclusive. Start with 0.95 for testing.
 * @param newVal  The new sample value to perform smoothing towards.
 *
 * @return The exponential smoothing function value.
 */
float proximityGetExp(float prevVal, float alpha, float newVal)
{
  return prevVal * alpha + newVal * (1 - alpha);
}

#if defined(MAXSONAR_ENABLED)
/* A proximity object for the maxSonar driver. */
proximity_t proximityMaxSonar;
#endif

#if !defined(PLATFORM_CF1)
/* A proximity object for the LPS25H driver. */
proximity_t proximityLPS25H;
#endif

#if defined(PROXIMITY_LOG_ENABLED)
/* Define a log group. */
LOG_GROUP_START(proximity)
#if defined(MAXSONAR_ENABLED)
LOG_ADD(LOG_FLOAT, maxsonarRaw, &proximityMaxSonar.distanceRaw)
LOG_ADD(LOG_FLOAT, maxsonarAvg, &proximityMaxSonar.distanceAvg)
LOG_ADD(LOG_FLOAT, maxsonarMed, &proximityMaxSonar.distanceMed)
LOG_ADD(LOG_FLOAT, maxsonarAccuracy, &proximityMaxSonar.accuracy)
#endif
#if !defined(PLATFORM_CF1)
LOG_ADD(LOG_FLOAT, lps25hRaw, &proximityLPS25H.distanceRaw)
LOG_ADD(LOG_FLOAT, lps25hAvg, &proximityLPS25H.distanceAvg)
LOG_ADD(LOG_FLOAT, lps25hMed, &proximityLPS25H.distanceMed)
LOG_ADD(LOG_FLOAT, lps25hAccuracy, &proximityLPS25H.accuracy)
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
  vTaskSetApplicationTaskTag(0, (void*)TASK_PROXIMITY_ID_NBR);

  //Wait for the system to be fully started to start proximity loop
  systemWaitStart();

  uint32_t lastWakeTime = xTaskGetTickCount();

#if defined(MAXSONAR_ENABLED)
  uint32_t maxSonarCounter = 0;
#endif

#if !defined(PLATFORM_CF1)
  uint32_t LPS25HCounter = 0;
#endif

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(PROXIMITY_TASK_FREQ));

#if defined(MAXSONAR_ENABLED)
    /* Read the MaxBotix sensor. */
    if(++maxSonarCounter >= PROXIMITY_MAXSONAR_SAMPLE_RATE_DIVIDER)
    {
      float msAccuracy;
      float msDistance = maxSonarReadDistance(MAXSONAR_MB1040_AN, &msAccuracy);
      proximityUpdate(&proximityMaxSonar, msDistance, proximityUpdAll, msAccuracy);
      maxSonarCounter = 0;
    }
#endif

#if !defined(PLATFORM_CF1)
    /* Read the LPS25H sensor. */
    if(++LPS25HCounter >= PROXIMITY_LPS25H_SAMPLE_RATE_DIVIDER)
    {
      float lpsPressure, lpsTemperature, lpsDistance;
      lps25hGetData(&lpsPressure, &lpsTemperature, &lpsDistance);
      proximityUpdate(&proximityLPS25H, lpsDistance, proximityUpdAll, 0);
      LPS25HCounter = 0;
    }
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
  proximityInit(&proximityMaxSonar, PROXIMITY_SWIN_MAX);
#endif

#if !defined(PLATFORM_CF1)
  proximityInit(&proximityLPS25H, PROXIMITY_SWIN_MAX);
#endif

#if defined(PROXIMITY_ENABLED)
  /* Only start the task if the proximity subsystem is enabled in conf.h */
  xTaskCreate(proximityTask, (const signed char * const)PROXIMITY_TASK_NAME,
              PROXIMITY_TASK_STACKSIZE, NULL, PROXIMITY_TASK_PRI, NULL);
#endif

  isInit = true;
}

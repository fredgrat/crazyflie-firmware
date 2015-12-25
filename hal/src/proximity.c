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
static void proximitySWCalcMed(proximity_t *proximity)
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
static void proximitySWCalcAvg(proximity_t *proximity)
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
 * This function calculates the variance of the sliding window.
 *
 * The proximity variance object's attribute 'variance' is updated with the variance value before returning.
 *
 * @note This function must be called after calling proximitySWCalcAvg().
 *
 * @param proximity Proximity object.
 */
static void proximitySWCalcVar(proximity_t *proximity)
{
  assert_param(proximity);
  assert_param(proximity->var);

  float proximityNewVar = 0;
  uint8_t n;
  for (n = 0; n < proximity->sWinSize; n++) {
    proximityNewVar += proximity->var->avgdiffsqr[n];
  }

  /* Update the variance calculations object with the variance value of the samples, considering the number of samples is sWinSize. */
  proximity->var->variance = proximityNewVar / (proximity->sWinSize - 1);
}

/**
 * This function adds a distance measurement sample to the sliding window, discarding the oldest sample.
 *
 * @param proximity Proximity object.
 * @param distance  The new sample to add to the sliding window.
 */
static void proximitySWAddSample(proximity_t *proximity, float distance)
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
 * This function adds a value to a separate, sliding window used for variance calculations.
 * The value added = (proximity->sWin[proximity->sWinSize - 1] - proximity->distanceAvg)^2.
 *
 * As for the other sliding window, the oldest value is discarded when adding the new one.
 *
 * @param proximity Proximity object.
 * @param distance  The new sample to add to the sliding window.
 */
static void proximitySWVarAddSample(proximity_t *proximity, float distance)
{
  assert_param(proximity);

  /* Discard oldest value, move remaining values one slot to the left. Moving only the sWinSize number of values. */
  memmove(&proximity->var->avgdiffsqr[0], &proximity->var->avgdiffsqr[1], (proximity->sWinSize - 1) * sizeof(float));

  /* Add the new calculated value in the last (right-most) slot. */
  float diff = proximity->sWin[proximity->sWinSize - 1] - proximity->distanceAvg;
  proximity->var->avgdiffsqr[proximity->sWinSize - 1] = diff * diff; /* Primitive square function. */
}

/**
 * Initialize a proximity object.
 *
 * @param proximity Proximity object.
 * @param sWinSize  The number of samples in the sliding window.
 */
void proximitySWInit(proximity_t *proximity, uint32_t sWinSize)
{
  assert_param(proximity);

  /* Now initialize the proximity object, including the sliding window. */
  proximitySWDeInit(proximity);
  proximity->sWinSize = sWinSize;
}

/**
 * Deinitialize a proximity object.
 *
 * @param proximity Proximity object.
 */
void proximitySWDeInit(proximity_t *proximity)
{
  assert_param(proximity);

  /* Zero the entire proximity object, including the sliding window of samples. */
  memset(proximity, 0, sizeof(proximity_t));
}

/**
 * Initialize a variance calculations object.
 *
 * The variance calculations object is tied to a proximity object to be updated by the proximityUpdate() function.
 */
void proximitySWVarInit(proximity_t *proximity, proximityVar_t *proximityVar)
{
  assert_param(proximity);
  assert_param(proximityVar);

  /* Zero the entire variance calculations object, including the avgdiffsqr array. */
  memset(proximityVar, 0, sizeof(proximityVar_t));

  proximity->var = proximityVar;
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
void proximitySWUpdate(proximity_t *proximity, float distance, proximityUpd_t updType, float accuracy)
{
  assert_param(proximity);

  /* The sample is added to the sliding window regardless. */
  proximitySWAddSample(proximity, distance);

  /* Check if a new average calculation shall be performed. */
  if((updType && proximityUpdAvg) > 0) {
    proximitySWCalcAvg(proximity);
  }

  /* Check if a new median calculation shall be performed. */
  if((updType && proximityUpdMed) > 0) {
    proximitySWCalcMed(proximity);
  }

  /* Only update accuracy if provided with a sensible value. If not, assume the same as for previous update, or updated otherwise. */
  if(accuracy > 0) {
    proximity->accuracy = accuracy;
  }

#if defined(PROXIMITY_LOG_ENABLED)
  /* If requested, and the variance object has been initialized, calculate the variance of the sliding window. */
  if(((updType && proximityUpdVar) > 0) && (NULL != proximity->var)) {
    proximitySWVarAddSample(proximity, distance);
    proximitySWCalcVar(proximity);
  }

  /* Also store the raw value for logging purposes. */
  proximity->distanceRaw = distance;
#endif
}

/**
 * Function returning the last proximity measurement.
 *
 * @param proximity Proximity object.
 *
 * @return The last, raw proximity measurement sample in the sliding window.
 */
float proximitySWGetRaw(proximity_t *proximity)
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
float proximitySWGetAvg(proximity_t *proximity)
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
float proximitySWGetMed(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->distanceMed;
}

/**
 * Function returning the result of the last variance calculation of the samples in the sliding window.
 *
 * @param proximity Proximity object.
 *
 * @return The result from the last variance calculation.
 */
float proximitySWGetVar(proximity_t *proximity)
{
  assert_param(proximity);
  assert_param(proximity->var);

  return proximity->var->variance;
}

/**
 * Function returning the accuracy of the last proximity measurement.
 *
 * @param proximity Proximity object.
 *
 * @return The accuracy of the last proximity measurement made.
 */
float proximitySWGetAccuracy(proximity_t *proximity)
{
  assert_param(proximity);

  return proximity->accuracy;
}

/**
 * Function to initialize the proximity Brown object.
 *
 * @param proximityBrown The proximity Brown object.
 * @param alpha          The smoothing factor alpha. Between 0 and 1 inclusive, where 1 means no smoothing.
 */
void proximityBrownInit(proximityBrown_t *proximityBrown, float alpha)
{
  assert_param(proximityBrown);

  proximityBrown->single_smoothed = 0;
  proximityBrown->double_smoothed = 0;
  proximityBrown->alpha = alpha;
}

/**
 * Calculate the simple exponential smoothing function.
 *
 * Typically this function is called by a task which uses sensor data to perform
 * stabilization and regulation, commonly running between 100-500Hz.
 *
 * Note that this function is not relevant to call when new samples are added (by using the
 * proximityUpdate() function). This function is used each time a new smoothed value is needed,
 * based on the existing measurement samples.
 *
 * @param proximityBrown The proximity Brown object.
 * @param measurement    The last measurement sample, to perform smoothing towards.
 *
 * @return The simple exponential smoothed value.
 */
float proximityBrownSimpleExp(proximityBrown_t *proximityBrown, float measurement)
{
  assert_param(proximityBrown);

  proximityBrown->single_smoothed = measurement * proximityBrown->alpha + proximityBrown->single_smoothed * (1 - proximityBrown->alpha);
  return proximityBrown->single_smoothed;
}

/**
 * Calculate the linear exponential smoothing function.
 *
 * Typically this function is called by a task which uses sensor data to perform
 * stabilization and regulation, commonly running between 100-500Hz.
 *
 * Note that this function is not relevant to call when new samples are added (by using the
 * proximityUpdate() function). This function is used each time a new smoothed value is needed,
 * based on the existing measurement samples.
 *
 * @param proximityBrown The proximity Brown object.
 * @param measurement    The last measurement sample, to perform smoothing towards.
 *
 * @return The linear exponential smoothed value.
 */
float proximityBrownLinearExp(proximityBrown_t *proximityBrown, float measurement)
{
  assert_param(proximityBrown);

  proximityBrown->single_smoothed = measurement * proximityBrown->alpha + proximityBrown->single_smoothed * (1 - proximityBrown->alpha);
  proximityBrown->double_smoothed = proximityBrown->single_smoothed * proximityBrown->alpha + proximityBrown->double_smoothed * (1 - proximityBrown->alpha);

  float est_a = 2 * proximityBrown->single_smoothed - proximityBrown->double_smoothed;
  float est_b = (proximityBrown->alpha / (1 - proximityBrown->alpha)) * (proximityBrown->single_smoothed - proximityBrown->double_smoothed);

  return est_a + est_b;
}

/**
 * Function to initialize the proximity Kalman object.
 *
 * @param proximityKalman The proximity Kalman object.
 */
void proximityKalmanInit(proximityKalman_t *proximityKalman, float control_scale, float initial_state_est, float initial_covariance, float control_noise, float measurement_noise)
{
  assert_param(proximityKalman);

  proximityKalman->control_scale = control_scale;
  proximityKalman->current_state_est = initial_state_est;
  proximityKalman->current_prob_est = initial_covariance;
  proximityKalman->control_noise = control_noise;
  proximityKalman->measurement_noise = measurement_noise;
}

float proximityKalmanUpdate(proximityKalman_t *proximityKalman)
{
  assert_param(proximityKalman);

  float kalman_gain = proximityKalman->predicted_prob_est * proximityKalman->innovation_covariance;
  proximityKalman->current_state_est = proximityKalman->predicted_state_est + kalman_gain * proximityKalman->innovation;
  proximityKalman->current_prob_est = (1 - kalman_gain) * proximityKalman->predicted_prob_est;

  return proximityKalman->current_state_est;
}

void proximityKalmanPredict(proximityKalman_t *proximityKalman, float control_value)
{
  assert_param(proximityKalman);

  proximityKalman->predicted_state_est = proximityKalman->current_state_est + (proximityKalman->control_scale * control_value);
  proximityKalman->predicted_prob_est = proximityKalman->current_prob_est + proximityKalman->control_noise;
}

void proximityKalmanObserve(proximityKalman_t *proximityKalman, float observation_value)
{
  assert_param(proximityKalman);

  proximityKalman->innovation = observation_value - proximityKalman->predicted_state_est;
  proximityKalman->innovation_covariance = proximityKalman->predicted_prob_est + proximityKalman->measurement_noise;
}

#if defined(MAXSONAR_ENABLED)
/* A proximity object for the maxSonar driver. */
proximity_t proximityMaxSonar;
proximityVar_t proximityVarMaxSonar;
#endif

#if !defined(PLATFORM_CF1)
/* A proximity object for the LPS25H driver. */
proximity_t proximityLPS25H;
proximityVar_t proximityVarLPS25H;
#endif

#if defined(PROXIMITY_LOG_ENABLED)
/* Define a log group. */
LOG_GROUP_START(proximity)
#if defined(MAXSONAR_ENABLED)
LOG_ADD(LOG_FLOAT, maxsonarRaw, &proximityMaxSonar.distanceRaw)
LOG_ADD(LOG_FLOAT, maxsonarAvg, &proximityMaxSonar.distanceAvg)
LOG_ADD(LOG_FLOAT, maxsonarMed, &proximityMaxSonar.distanceMed)
LOG_ADD(LOG_FLOAT, maxsonarAccuracy, &proximityMaxSonar.accuracy)
LOG_ADD(LOG_FLOAT, maxsonarVar, &proximityVarMaxSonar.variance)
#endif
#if !defined(PLATFORM_CF1)
LOG_ADD(LOG_FLOAT, lps25hRaw, &proximityLPS25H.distanceRaw)
LOG_ADD(LOG_FLOAT, lps25hAvg, &proximityLPS25H.distanceAvg)
LOG_ADD(LOG_FLOAT, lps25hMed, &proximityLPS25H.distanceMed)
LOG_ADD(LOG_FLOAT, lps25hAccuracy, &proximityLPS25H.accuracy)
LOG_ADD(LOG_FLOAT, lps25hVar, &proximityVarLPS25H.variance)
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
      proximitySWUpdate(&proximityMaxSonar, msDistance, proximityUpdAll, msAccuracy);
      maxSonarCounter = 0;
    }
#endif

#if !defined(PLATFORM_CF1)
    /* Read the LPS25H sensor. */
    if(++LPS25HCounter >= PROXIMITY_LPS25H_SAMPLE_RATE_DIVIDER)
    {
      float lpsPressure, lpsTemperature, lpsDistance;
      lps25hGetData(&lpsPressure, &lpsTemperature, &lpsDistance);
      proximitySWUpdate(&proximityLPS25H, lpsDistance, proximityUpdAll, 0);
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
  proximitySWInit(&proximityMaxSonar, PROXIMITY_SWIN_MAX);
  proximitySWVarInit(&proximityMaxSonar, &proximityVarMaxSonar);
#endif

#if !defined(PLATFORM_CF1)
  proximitySWInit(&proximityLPS25H, PROXIMITY_SWIN_MAX);
  proximitySWVarInit(&proximityLPS25H, &proximityVarLPS25H);
#endif

#if defined(PROXIMITY_ENABLED)
  /* Only start the task if the proximity subsystem is enabled in conf.h */
  xTaskCreate(proximityTask, (const signed char * const)PROXIMITY_TASK_NAME,
              PROXIMITY_TASK_STACKSIZE, NULL, PROXIMITY_TASK_PRI, NULL);
#endif

  isInit = true;
}

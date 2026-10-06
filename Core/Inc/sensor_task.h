#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * SHT41 measurement cycle:
 *   1. power sensor on
 *   2. initialize I2C / reset sensor
 *   3. measure temperature
 *   4. send temperature through BLE notification
 *   5. power sensor off
 *   6. wait 5 seconds
 *
 * The task itself is executed by the STM32WB sequencer, so the MCU can
 * enter STOP2 between events.
 */

void SensorTask_Init(void);
void SensorTask_Start(void);
void SensorTask_Stop(void);
void SensorTask_RequestMeasurement(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TASK_H */

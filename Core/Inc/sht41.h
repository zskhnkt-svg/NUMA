#ifndef SHT41_H
#define SHT41_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include "i2c.h"

#define SHT41_I2C_ADDRESS       (0x44U << 1)
#define SHT41_CMD_SOFT_RESET    0x94U
#define SHT41_CMD_MEASURE_HIGH  0xFDU

typedef struct
{
  float temperature;
  float humidity;
} SHT41_Data_t;

HAL_StatusTypeDef SHT41_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SHT41_Read(I2C_HandleTypeDef *hi2c,
                             SHT41_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* SHT41_H */

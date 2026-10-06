/**
 * @file sht41.c
 * @brief Minimal SHT41 I2C driver.
 */

#include "sht41.h"

static uint8_t SHT41_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFF;

  for (uint8_t i = 0; i < length; ++i)
  {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U)
                           : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

HAL_StatusTypeDef SHT41_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t command = SHT41_CMD_SOFT_RESET;

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      hi2c,
      SHT41_I2C_ADDRESS,
      &command,
      1,
      100);

  if (status == HAL_OK)
  {
    /*
     * SHT41 needs at least 1 ms after a soft reset before another command.
     */
    HAL_Delay(1);
  }

  return status;
}

HAL_StatusTypeDef SHT41_Read(I2C_HandleTypeDef *hi2c,
                             SHT41_Data_t *data)
{
  if (data == NULL)
  {
    return HAL_ERROR;
  }

  uint8_t command = SHT41_CMD_MEASURE_HIGH;
  uint8_t buffer[6];

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      hi2c,
      SHT41_I2C_ADDRESS,
      &command,
      1,
      100);

  if (status != HAL_OK)
  {
    return status;
  }

  /*
   * High precision measurement requires up to about 9 ms.
   * HAL_Delay() sleeps using WFI in this project.
   */
  HAL_Delay(10);

  status = HAL_I2C_Master_Receive(
      hi2c,
      SHT41_I2C_ADDRESS,
      buffer,
      sizeof(buffer),
      100);

  if (status != HAL_OK)
  {
    return status;
  }

  if (SHT41_Crc8(&buffer[0], 2) != buffer[2] ||
      SHT41_Crc8(&buffer[3], 2) != buffer[5])
  {
    return HAL_ERROR;
  }

  uint16_t raw_temperature = ((uint16_t)buffer[0] << 8) | buffer[1];
  uint16_t raw_humidity    = ((uint16_t)buffer[3] << 8) | buffer[4];

  /*
   * SHT41 conversion formulas from the datasheet.
   */
  data->temperature = -45.0f +
                      (175.0f * (float)raw_temperature / 65535.0f);

  data->humidity = -6.0f +
                   (125.0f * (float)raw_humidity / 65535.0f);

  /*
   * Physical RH range is 0...100 %. The raw conversion can produce a
   * slightly wider mathematical range at the edges.
   */
  if (data->humidity < 0.0f)
  {
    data->humidity = 0.0f;
  }
  else if (data->humidity > 100.0f)
  {
    data->humidity = 100.0f;
  }

  return HAL_OK;
}

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_seconds;
    int16_t  average_x100;
    int16_t  minimum_x100;
    int16_t  maximum_x100;
    uint16_t sample_count;
} DataLogger_Record_t;

HAL_StatusTypeDef DataLogger_Init(void);
HAL_StatusTypeDef DataLogger_Append(uint32_t timestamp_seconds,
                                    int16_t average_x100,
                                    int16_t minimum_x100,
                                    int16_t maximum_x100,
                                    uint16_t sample_count);
HAL_StatusTypeDef DataLogger_Read(uint32_t index,
                                  DataLogger_Record_t *record);
uint32_t DataLogger_GetRecordCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGER_H */

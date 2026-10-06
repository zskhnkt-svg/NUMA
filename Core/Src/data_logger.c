/**
 * @file data_logger.c
 * @brief Circular temperature journal in internal STM32WB Flash.
 */

#include "data_logger.h"
#include "stm32wbxx_hal.h"

#include <stddef.h>
#include <string.h>

#define DATA_LOG_MAGIC          0x544C4F47UL
#define DATA_LOG_FLASH_PAGE     4096U
#define DATA_LOG_RECORD_SIZE    32U
#define DATA_LOG_MAX_RECORDS    ((uint32_t)(((uint32_t)&__data_log_end - \
                                             (uint32_t)&__data_log_start) / \
                                            DATA_LOG_RECORD_SIZE))

extern uint8_t __data_log_start;
extern uint8_t __data_log_end;

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t timestamp_seconds;
    int16_t  average_x100;
    int16_t  minimum_x100;
    int16_t  maximum_x100;
    uint16_t sample_count;
    uint16_t crc;
    uint16_t reserved;
    uint32_t reserved2;
    uint32_t reserved3[1];
} DataLogFlashRecord_t;

_Static_assert(sizeof(DataLogFlashRecord_t) == DATA_LOG_RECORD_SIZE,
               "DataLogFlashRecord_t must be 32 bytes");

static uint32_t s_write_index;
static uint32_t s_next_sequence;
static uint32_t s_record_count;
static uint8_t s_initialized;

static uint16_t DataLogger_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- != 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                              : (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

static uint32_t DataLogger_Address(uint32_t index)
{
    return (uint32_t)&__data_log_start + index * DATA_LOG_RECORD_SIZE;
}

static uint8_t DataLogger_IsEmpty(uint32_t address)
{
    const uint32_t *words = (const uint32_t *)address;

    for (uint8_t index = 0U; index < 8U; ++index)
    {
        if (words[index] != 0xFFFFFFFFUL)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t DataLogger_IsValid(const DataLogFlashRecord_t *record)
{
    if (record->magic != DATA_LOG_MAGIC || record->sample_count == 0U)
    {
        return 0U;
    }

    return record->crc == DataLogger_Crc16((const uint8_t *)record,
                                           offsetof(DataLogFlashRecord_t, crc));
}

static HAL_StatusTypeDef DataLogger_EraseAll(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    uint32_t first_page = ((uint32_t)&__data_log_start - FLASH_BASE) /
                          DATA_LOG_FLASH_PAGE;
    uint32_t page_count = ((uint32_t)&__data_log_end -
                           (uint32_t)&__data_log_start) /
                          DATA_LOG_FLASH_PAGE;
    HAL_StatusTypeDef status;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = first_page;
    erase.NbPages = page_count;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef DataLogger_Init(void)
{
    const DataLogFlashRecord_t *record;
    uint32_t newest_index = 0U;
    uint32_t newest_sequence = 0U;
    uint32_t valid_count = 0U;
    uint8_t found = 0U;

    for (uint32_t index = 0U; index < DATA_LOG_MAX_RECORDS; ++index)
    {
        record = (const DataLogFlashRecord_t *)DataLogger_Address(index);
        if (DataLogger_IsValid(record))
        {
            ++valid_count;
            if (!found || record->sequence > newest_sequence)
            {
                found = 1U;
                newest_sequence = record->sequence;
                newest_index = index;
            }
        }
    }

    if (!found)
    {
        s_write_index = 0U;
        s_next_sequence = 0U;
    }
    else
    {
        s_write_index = newest_index + 1U;
        s_next_sequence = newest_sequence + 1U;

        if (s_write_index >= DATA_LOG_MAX_RECORDS)
        {
            s_write_index = 0U;
        }

        if (!DataLogger_IsEmpty(DataLogger_Address(s_write_index)))
        {
            if (DataLogger_EraseAll() != HAL_OK)
            {
                return HAL_ERROR;
            }
            s_write_index = 0U;
            s_next_sequence = 0U;
            valid_count = 0U;
        }
    }

    s_record_count = valid_count;
    s_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef DataLogger_Append(uint32_t timestamp_seconds,
                                    int16_t average_x100,
                                    int16_t minimum_x100,
                                    int16_t maximum_x100,
                                    uint16_t sample_count)
{
    DataLogFlashRecord_t record = {0};
    uint32_t address;
    HAL_StatusTypeDef status = HAL_OK;

    if (!s_initialized || sample_count == 0U)
    {
        return HAL_ERROR;
    }

    if (s_write_index >= DATA_LOG_MAX_RECORDS)
    {
        if (DataLogger_EraseAll() != HAL_OK)
        {
            return HAL_ERROR;
        }
        s_write_index = 0U;
        s_next_sequence = 0U;
        s_record_count = 0U;
    }

    record.magic = DATA_LOG_MAGIC;
    record.sequence = s_next_sequence++;
    record.timestamp_seconds = timestamp_seconds;
    record.average_x100 = average_x100;
    record.minimum_x100 = minimum_x100;
    record.maximum_x100 = maximum_x100;
    record.sample_count = sample_count;
    record.reserved = 0xFFFFU;
    record.reserved2 = 0xFFFFFFFFUL;
    record.reserved3[0] = 0xFFFFFFFFUL;
    record.crc = DataLogger_Crc16((const uint8_t *)&record,
                                  offsetof(DataLogFlashRecord_t, crc));

    address = DataLogger_Address(s_write_index);

    HAL_FLASH_Unlock();
    for (uint32_t offset = 0U; offset < sizeof(record); offset += 8U)
    {
        uint64_t value;
        memcpy(&value, ((const uint8_t *)&record) + offset, sizeof(value));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   address + offset,
                                   value);
        if (status != HAL_OK)
        {
            break;
        }
    }
    HAL_FLASH_Lock();

    if (status == HAL_OK)
    {
        ++s_write_index;
        ++s_record_count;
    }

    return status;
}

HAL_StatusTypeDef DataLogger_Read(uint32_t index,
                                  DataLogger_Record_t *record)
{
    const DataLogFlashRecord_t *stored;

    if (!s_initialized || record == NULL || index >= s_record_count)
    {
        return HAL_ERROR;
    }

    stored = (const DataLogFlashRecord_t *)DataLogger_Address(index);
    if (!DataLogger_IsValid(stored))
    {
        return HAL_ERROR;
    }

    record->sequence = stored->sequence;
    record->timestamp_seconds = stored->timestamp_seconds;
    record->average_x100 = stored->average_x100;
    record->minimum_x100 = stored->minimum_x100;
    record->maximum_x100 = stored->maximum_x100;
    record->sample_count = stored->sample_count;

    return HAL_OK;
}

uint32_t DataLogger_GetRecordCount(void)
{
    return s_record_count;
}
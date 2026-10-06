/**
 * @file sensor_task.c
 * @brief Autonomous SHT41 temperature sampling and BLE notification task.
 *
 * The sensor is measured every minute independently of BLE. The latest
 * temperature is sent only while the phone is connected and subscribed to
 * the notification characteristic.
 */

#include "sensor_task.h"
#include "app_ble.h"

/* STM32WB middleware and the application use the same PAUSE macro. */
#ifdef PAUSE
#undef PAUSE
#endif
#include "app_common.h"

#ifdef PAUSE
#undef PAUSE
#endif
#include "ble.h"
#include "dbg_trace.h"
#include "data_logger.h"
#include "hw_if.h"
#include "i2c.h"
#include "p2p_server_app.h"
#include "p2p_stm.h"
#include "rtc.h"
#include "sht41.h"
#include "stm32_seq.h"

#include <stdio.h>

#define SENSOR_POWER_GPIO_PORT      GPIOA
#define SENSOR_POWER_PIN            GPIO_PIN_4
#define SENSOR_POWER_ON_LEVEL       GPIO_PIN_RESET /* P-MOSFET: LOW = ON */
#define SENSOR_POWER_OFF_LEVEL      GPIO_PIN_SET   /* P-MOSFET: HIGH = OFF */

#define SENSOR_PERIOD_MS            60000U
#define SENSOR_POWER_UP_DELAY_MS    2U
#define SENSOR_NOTIFY_LED_MS        15U
#define LOGGER_SAMPLES_PER_RECORD   5U
#define HISTORY_SEND_INTERVAL_MS    100U
#define HISTORY_PAYLOAD_MAX         60U

#define SENSOR_PERIOD_TICKS \
    ((SENSOR_PERIOD_MS * 1000U + (CFG_TS_TICK_VAL / 2U)) / CFG_TS_TICK_VAL)
#define HISTORY_SEND_TICKS \
    ((HISTORY_SEND_INTERVAL_MS * 1000U + (CFG_TS_TICK_VAL / 2U)) / CFG_TS_TICK_VAL)

extern volatile uint8_t Notification_Status;

static uint8_t s_sensor_timer_id;
static uint8_t s_history_timer_id;
static uint8_t s_initialized;
static uint8_t s_history_active;
static uint32_t s_history_index;
static uint8_t s_payload_valid;
static uint8_t s_payload_length;
static char s_payload[16];
static int32_t s_temperature_sum_x100;
static int16_t s_temperature_min_x100;
static int16_t s_temperature_max_x100;
static uint8_t s_temperature_count;

static void Sensor_I2C_LinesToAF(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void Sensor_PowerOn(void)
{
    Sensor_I2C_LinesToAF();

    HAL_GPIO_WritePin(SENSOR_POWER_GPIO_PORT,
                      SENSOR_POWER_PIN,
                      SENSOR_POWER_ON_LEVEL);

    HAL_Delay(SENSOR_POWER_UP_DELAY_MS);
    MX_I2C1_Init();
}

static void Sensor_PowerOff(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_I2C_DeInit(&hi2c1);

    /* Hold I2C lines LOW to prevent back-powering the unpowered sensor. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SENSOR_POWER_GPIO_PORT,
                      SENSOR_POWER_PIN,
                      SENSOR_POWER_OFF_LEVEL);
}

static int16_t Sensor_TemperatureToCenti(float temperature)
{
    if (temperature >= 0.0f)
    {
        return (int16_t)(temperature * 100.0f + 0.5f);
    }

    return (int16_t)(temperature * 100.0f - 0.5f);
}

static void Sensor_SetTemperaturePayload(int16_t temperature_centi)
{
    int32_t whole = temperature_centi / 100;
    int32_t fraction = temperature_centi % 100;
    int length;

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    length = snprintf(s_payload,
                      sizeof(s_payload),
                      "T=%ld.%02ld",
                      (long)whole,
                      (long)fraction);

    if (length > 0 && length < (int)sizeof(s_payload))
    {
        s_payload_length = (uint8_t)length;
        s_payload_valid = 1U;
    }
}

static uint32_t Sensor_GetTimestamp(void)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    return (((uint32_t)date.Month * 32U + date.Date) * 24U + time.Hours) * 3600U +
           (uint32_t)time.Minutes * 60U + time.Seconds;
}

static void Sensor_UpdateOfflineLog(int16_t temperature_centi)
{
    if (s_temperature_count == 0U)
    {
        s_temperature_min_x100 = temperature_centi;
        s_temperature_max_x100 = temperature_centi;
    }
    else
    {
        if (temperature_centi < s_temperature_min_x100)
        {
            s_temperature_min_x100 = temperature_centi;
        }
        if (temperature_centi > s_temperature_max_x100)
        {
            s_temperature_max_x100 = temperature_centi;
        }
    }

    s_temperature_sum_x100 += temperature_centi;
    ++s_temperature_count;

    if (s_temperature_count >= LOGGER_SAMPLES_PER_RECORD)
    {
        int16_t average = (int16_t)(s_temperature_sum_x100 /
                                    (int32_t)s_temperature_count);

        /* Always keep a five-minute backup, even while BLE is connected. */
        (void)DataLogger_Append(Sensor_GetTimestamp(),
                                average,
                                s_temperature_min_x100,
                                s_temperature_max_x100,
                                s_temperature_count);

        s_temperature_sum_x100 = 0;
        s_temperature_count = 0U;
    }
}

static void Sensor_CentiToText(int16_t value, char *text, size_t text_size)
{
    int32_t whole = value / 100;
    int32_t fraction = value % 100;

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    (void)snprintf(text, text_size, "%ld.%02ld", (long)whole, (long)fraction);
}

static uint8_t Sensor_SendHistoryRecord(void)
{
    DataLogger_Record_t record;
    char average[12];
    char minimum[12];
    char maximum[12];
    char payload[64];
    int length;
    tBleStatus status;

    if (!s_history_active || !Notification_Status)
    {
        s_history_active = 0U;
        return 0U;
    }

    if (s_history_index >= DataLogger_GetRecordCount())
    {
        s_history_active = 0U;
        return 0U;
    }

    if (DataLogger_Read(s_history_index, &record) != HAL_OK)
    {
        s_history_active = 0U;
        return 0U;
    }

    Sensor_CentiToText(record.average_x100, average, sizeof(average));
    Sensor_CentiToText(record.minimum_x100, minimum, sizeof(minimum));
    Sensor_CentiToText(record.maximum_x100, maximum, sizeof(maximum));

    length = snprintf(payload,
                      sizeof(payload),
                      "H,%lu,%lu,%s,%s,%s,%u",
                      (unsigned long)record.sequence,
                      (unsigned long)record.timestamp_seconds,
                      average,
                      minimum,
                      maximum,
                      record.sample_count);

    if (length <= 0 || length > (int)HISTORY_PAYLOAD_MAX ||
        length >= (int)sizeof(payload))
    {
        s_history_active = 0U;
        return 0U;
    }

    status = P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID,
                                      (uint8_t *)payload,
                                      (uint8_t)length);

    if (status == BLE_STATUS_SUCCESS)
    {
        ++s_history_index;
        HW_TS_Start(s_history_timer_id, HISTORY_SEND_TICKS);
    }

    return 1U;
}

static void Sensor_SendTemperature(void)
{
    tBleStatus status;

    if (!s_payload_valid || !Notification_Status)
    {
        return;
    }

    status = P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID,
                                      (uint8_t *)s_payload,
                                      s_payload_length);

    if (status == BLE_STATUS_SUCCESS)
    {
        s_payload_valid = 0U;
        APP_BLE_Led_Blink(SENSOR_NOTIFY_LED_MS);
    }
    else
    {
        APP_DBG_MSG("BLE temperature notification failed: 0x%02X\r\n",
                    status);
    }
}

static void SensorTask_Execute(void)
{
    SHT41_Data_t data;

    if (s_history_active)
    {
        if (Sensor_SendHistoryRecord())
        {
            return;
        }
    }

    HAL_StatusTypeDef status;

    Sensor_PowerOn();

    status = SHT41_Init(&hi2c1);
    if (status != HAL_OK)
    {
        Sensor_PowerOff();
        APP_DBG_MSG("SHT41 reset failed: %d\r\n", (int)status);
        return;
    }

    status = SHT41_Read(&hi2c1, &data);
    Sensor_PowerOff();

    if (status != HAL_OK)
    {
        APP_DBG_MSG("SHT41 read failed: %d\r\n", (int)status);
        return;
    }

    int16_t temperature_centi = Sensor_TemperatureToCenti(data.temperature);

    Sensor_UpdateOfflineLog(temperature_centi);
    Sensor_SetTemperaturePayload(temperature_centi);
    Sensor_SendTemperature();
}

static void HistoryTimer_Callback(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_SENSOR_MEASURE_ID,
                     CFG_SCH_PRIO_0);
}

static void SensorTimer_Callback(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_SENSOR_MEASURE_ID,
                     CFG_SCH_PRIO_0);
}

void SensorTask_Init(void)
{
    UTIL_SEQ_RegTask(1U << CFG_TASK_SENSOR_MEASURE_ID,
                     UTIL_SEQ_RFU,
                     SensorTask_Execute);

    HW_TS_Create(CFG_TIM_PROC_ID_ISR,
                 &s_sensor_timer_id,
                 hw_ts_Repeated,
                 SensorTimer_Callback);

    HW_TS_Create(CFG_TIM_PROC_ID_ISR,
                 &s_history_timer_id,
                 hw_ts_SingleShot,
                 HistoryTimer_Callback);

    s_initialized = 1U;
    s_payload_valid = 0U;
    s_history_active = 0U;
    s_history_index = 0U;
    s_temperature_sum_x100 = 0;
    s_temperature_count = 0U;

    if (DataLogger_Init() != HAL_OK)
    {
        APP_DBG_MSG("Data logger initialization failed\r\n");
    }

    Sensor_PowerOff();
    HW_TS_Start(s_sensor_timer_id, SENSOR_PERIOD_TICKS);
}

void SensorTask_Start(void)
{
    SensorTask_RequestMeasurement();
}

void SensorTask_Stop(void)
{
    /* Sampling remains autonomous after BLE disconnection. */
    Sensor_PowerOff();
}

void SensorTask_RequestMeasurement(void)
{
    if (s_initialized)
    {
        s_history_index = 0U;
        s_history_active = (DataLogger_GetRecordCount() > 0U) ? 1U : 0U;
        UTIL_SEQ_SetTask(1U << CFG_TASK_SENSOR_MEASURE_ID,
                         CFG_SCH_PRIO_0);
    }
}
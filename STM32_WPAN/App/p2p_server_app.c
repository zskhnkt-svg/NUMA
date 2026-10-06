/**
 * @file p2p_server_app.c
 * @brief BLE application callbacks used by the temperature sensor.
 */

#include "main.h"
#include "app_ble.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "sensor_task.h"

volatile uint8_t Notification_Status = 0;

void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
  switch (pNotification->P2P_Evt_Opcode)
  {
    case P2PS_STM__NOTIFY_ENABLED_EVT:
      Notification_Status = 1;
      APP_DBG_MSG("BLE: temperature notifications enabled\r\n");
      SensorTask_RequestMeasurement();
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
      Notification_Status = 0;
      APP_DBG_MSG("BLE: temperature notifications disabled\r\n");
      break;

    case P2PS_STM_WRITE_EVT:
      /*
       * The old project used this event for LED/sleep commands.
       * They are intentionally removed: power management is now automatic
       * and controlled by the BLE connection/notification state.
       */
      break;

    default:
      break;
  }
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
  switch (pNotification->P2P_Evt_Opcode)
  {
    case PEER_CONN_HANDLE_EVT:
      APP_DBG_MSG("BLE: connected\r\n");
      break;

    case PEER_DISCON_HANDLE_EVT:
      Notification_Status = 0;
      APP_DBG_MSG("BLE: disconnected\r\n");
      break;

    default:
      break;
  }
}

void P2PS_APP_Init(void)
{
  /* The generated SVCCTL_Init() initializes P2PS_STM. */
}

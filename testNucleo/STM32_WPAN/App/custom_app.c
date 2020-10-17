/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/custom_app.c
 * Description        : Custom Example Application (Server)
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* LONG_BME */
/* USER CODE BEGIN CUSTOM_APP_Context_t */

/* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

uint8_t SecureReadData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  /* LONG_BME */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

/* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch(pNotification->Custom_Evt_Opcode)
  {
/* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

/* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

  /* LONG_BME */
    case CUSTOM_STM_T_READ_EVT:
/* USER CODE BEGIN CUSTOM_STM_T_READ_EVT */

/* USER CODE END CUSTOM_STM_T_READ_EVT */
      break;

    case CUSTOM_STM_P_READ_EVT:
/* USER CODE BEGIN CUSTOM_STM_P_READ_EVT */

/* USER CODE END CUSTOM_STM_P_READ_EVT */
      break;

    case CUSTOM_STM_H_READ_EVT:
/* USER CODE BEGIN CUSTOM_STM_H_READ_EVT */

/* USER CODE END CUSTOM_STM_H_READ_EVT */
      break;

    default:
/* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

/* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

/* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN CUSTOM_APP_Notification_1 */

/* USER CODE END CUSTOM_APP_Notification_1 */

  switch(pNotification->Custom_Evt_Opcode)
  {
/* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

/* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
  case CUSTOM_CONN_HANDLE_EVT :
/* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

/* USER CODE END CUSTOM_CONN_HANDLE_EVT */
    break;

    case CUSTOM_DISCON_HANDLE_EVT :
/* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

/* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN CUSTOM_APP_Notification_default */

/* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

/* USER CODE BEGIN CUSTOM_APP_Notification_2 */

/* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
/* USER CODE BEGIN CUSTOM_APP_Init */
	tBleStatus ret;
	//ret = hci_reset();
	//ret = aci_gatt_init();
	Service_UUID_t BME_uuid;
		BME_uuid.Service_UUID_16 = 0x181a; // Environmental sensing
		uint16_t BMEserviceh;
		Char_UUID_t Temp_uuid;
		Temp_uuid.Char_UUID_16 = 0x0001;
		uint16_t Temp_charh;
		aci_gatt_add_service(0x02, &BME_uuid, 0x01, 3, &BMEserviceh);
		aci_gatt_add_char(BMEserviceh, 0x01, &Temp_uuid, 1, CHAR_PROP_BROADCAST, 0, 0, 07, 0, Temp_charh);
		aci_gatt_update_char_value(BMEserviceh, Temp_charh, 0, 1, 0x99);
	uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
	//ret = aci_gap_init(0x02, 0x00, 8, &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);
	uint8_t* broadcastedData = "HELLOWORLD";
	uint8_t* name = malloc(5);
	name[0] = 0x09;
	memcpy(name+1, "SUUS", 4);
	//name[1] = "SUUS";
	//aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(broadcastedData), broadcastedData);
	//
	//ret = hci_le_set_advertising_data(strlen(broadcastedData), (uint8_t*)broadcastedData);
	//ret = hci_le_set_advertise_enable(1);
	//ret = aci_gap_update_adv_data(strlen(broadcastedData), broadcastedData);
	//aci_gap_set_discoverable(ADV_NONCONN_IND, 100, 100, 0x01, NO_WHITE_LIST_USE,
		//	5, name, 0, NULL, 0, 0);


	Whitelist_Entry_t whitelist[1];
	memset(whitelist[0].Peer_Address, 0, 6);
	whitelist[0].Peer_Address_Type = 0;
	uint8_t data = 0xAA;
	ret = aci_gap_set_broadcast_mode(0x00A0, 0x00A1, 0x03, 0x01, 1, &data, 1, whitelist);
	/* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

  /* LONG_BME */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

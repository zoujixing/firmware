/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * BLE Proximity Reporter Sample Application
 *
 * Features demonstrated
 *  - WICED BT GATT server APIs
 *
 * On startup this demo:
 *  - Initializes the GATT subsystem
 *  - Begins advertising
 *  - Waits for GATT clients to connect
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 */

#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "ble_provision_gatt_db.h"
#include "ble_provision.h"
#include "wlan_hal.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define SCAN_RECORD_MAX_NUM		20
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum
{
	GapStatus_Connected,
	GapStatus_Disconnect,
}GapStatus;

typedef struct
{
	wiced_bool_t             device_configured;
	wiced_config_ap_entry_t  ap_entry;
} temp_config_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void ble_provision_init(void);

static wiced_bt_dev_status_t ble_provision_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request);
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request);


/******************************************************
 *               Variable Definitions
 ******************************************************/

/* BLE connect variable */
static uint16_t 	connect_id;
static GapStatus  	connect_status = GapStatus_Disconnect;

/* GATT attrIbute values */
static uint32_t    	peripheral_gatt_attribute_service_changed = 0;
static uint16_t    	peripheral_gatt_generic_access_appearance = 0;

static uint8_t  	provision_ssid[20];
static uint8_t		provision_ssid_len;
static uint8_t     	provision_security;
static uint8_t 		provision_password[20];
static uint8_t		provision_password_len;
static uint8_t 		provision_action;

static temp_config_t temp_config;

/* Stack and buffer pool configuration tables */
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************
 *               Function Definitions
 ******************************************************/
void bt_statck_init(void)
{
	/* Initialize Bluetooth controller and host stack */
	wiced_bt_stack_init( ble_provision_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

void bt_loop_event(void)
{
	//taskYIELD();
	wiced_rtos_delay_milliseconds( 10 );
}

/* Initialize peripheral */
static void ble_provision_init(void)
{
	uint8_t buf[16] = {0xAA,0xBB,0x00,0x00,0xDF,0xFB,0x48,0xD2,0xB0,0x60,0xD0,0xF5,0xA7,0x10,0x96,0xE0};

	wiced_bt_ble_advert_data_t adv_data;
    /* Set advertising data: device name and discoverable flag */

    adv_data.flag = 0x06;

    adv_data.p_services_128b->list_cmpl = WICED_TRUE;
    memcpy(adv_data.p_services_128b->uuid128, buf, 16);

    wiced_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_FLAGS|BTM_BLE_ADVERT_BIT_DEV_NAME, &adv_data);

    /* Register for gatt event notifications */
    wiced_bt_gatt_register(&ble_provision_gatt_cback);

    /* Initialize GATT database */
    wiced_bt_gatt_db_init ((uint8_t *)gatt_db, gatt_db_size);

    /* Enable Bluetooth LE advertising and connectability */

    /* start LE advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

/******************************************************
 *             Callback Function Definitions
 ******************************************************/
/* Bluetooth management event handler */
static wiced_bt_dev_status_t ble_provision_management_cback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth controller and host stack enabled */
        if (p_event_data->enabled.status == WICED_BT_SUCCESS)
        {
            /* Enable peripheral */
            ble_provision_init();
        }
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* adv state change callback */
        if (BTM_BLE_ADVERT_OFF == p_event_data->ble_advert_state_changed)
        {
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        }
        break;

    default:
        break;
    }

    return (status);
}

/* GATT event handler */
static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch (event)
	{
		case GATT_CONNECTION_STATUS_EVT:
			/* GATT connection status change */
			if (p_event_data->connection_status.connected)
			{
				connect_status = GapStatus_Connected;
				connect_id = p_event_data->connection_status.conn_id;

				/* Disable connectability. */
				wiced_bt_start_advertisements (BTM_BLE_ADVERT_OFF, 0, NULL);
			}
			else
			{
				connect_status = GapStatus_Disconnect;
				connect_id = 0x0000;
				/* Connection released. Re-enable BLE connectability. */
				wiced_bt_start_advertisements (BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
			}
			break;

		case GATT_ATTRIBUTE_REQUEST_EVT:
			/* GATT attribute read/write request */
			if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE)
			{
				status = ble_provision_gatt_write_request(&p_event_data->attribute_request.data.write_req);
			}
			else if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_READ)
			{
				status = ble_provision_gatt_read_request(&p_event_data->attribute_request.data.read_req);

			}
			break;

		default:
			break;
    }

    return (status);
}

/* Handler for attrubute write requests */
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request)
{
	uint8_t new_value[20];
	uint8_t len = p_write_request->val_len;

	wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    memcpy(new_value, (uint8_t *)p_write_request->p_val, len);

    switch (p_write_request->handle)
    {
		case HDLC_PROVISION_SSID_VALUE:
			if(len <= 20)
			{
				memset(provision_ssid, '0', 20);
				memcpy(provision_ssid, new_value, len);
				provision_ssid_len = len;

				temp_config.ap_entry.details.SSID.length = provision_ssid_len;
				memcpy(&temp_config.ap_entry.details.SSID.value, provision_ssid, provision_ssid_len);
				temp_config.ap_entry.details.SSID.value[temp_config.ap_entry.details.SSID.length] = 0;
			}
			else
			{
				status = WICED_BT_GATT_INVALID_ATTR_LEN;
			}
			break;

        case HDLC_PROVISION_SECURITY_VALUE:
        	if(len == 0x01)
        	{
        		wiced_security_t wiced_security;
        		provision_security = new_value[0];

        		if(provision_security == 0x00)
        		{
        			wiced_security = WICED_SECURITY_OPEN;
        		}
        		else if(provision_security == 0x01)
        		{
        			wiced_security = WICED_SECURITY_WPA_AES_PSK;
        		}
        		else if(provision_security == 0x02)
        		{
        			wiced_security = WICED_SECURITY_WPA2_AES_PSK;
        		}
        		else
        		{
        			wiced_security = WICED_SECURITY_WEP_PSK;
        		}
        		memcpy(&temp_config.ap_entry.details.security, &wiced_security, sizeof(wiced_security_t));
        	}
        	else
        	{
        		status = WICED_BT_GATT_INVALID_ATTR_LEN;
        	}
            break;

        case HDLC_PROVISION_PASSWORD_VALUE:
        	if(len <= 20)
			{
        		memset(provision_password, 0, 20);
				memcpy(provision_password, new_value, len);
				provision_password_len = len;

				memcpy(temp_config.ap_entry.security_key, provision_password, provision_password_len);
				temp_config.ap_entry.security_key_length = provision_password_len;
				temp_config.ap_entry.security_key[temp_config.ap_entry.security_key_length] = 0;
			}
			else
			{
				status = WICED_BT_GATT_INVALID_ATTR_LEN;
			}
        	break;

        case HDLC_PROVISION_ACTION_VALUE:
        	if(len == 0x01)
			{
        		provision_action = new_value[0];
				if(provision_action == 0x01)
				{
					temp_config.device_configured = WICED_TRUE;

					/* Write received credentials into DCT */
					wiced_dct_write( &temp_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(temp_config_t) );

					HAL_WLAN_notify_simple_config_done();
				}
			}
			else
			{
				status = WICED_BT_GATT_INVALID_ATTR_LEN;
			}
        	break;

		default:
			status = WICED_BT_GATT_WRITE_NOT_PERMIT;
			break;
    }

    return (status);
}

/* Handler for attrubute read requests */
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    void *p_attribute_value_source;
    uint16_t attribute_value_length = 0;

    switch (p_read_request->handle)
    {
		case HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE:
			p_attribute_value_source    = &peripheral_gatt_attribute_service_changed;
			attribute_value_length      = sizeof(peripheral_gatt_attribute_service_changed);
			break;

		case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
			p_attribute_value_source    = (void *)wiced_bt_cfg_settings.device_name;
			attribute_value_length      = strlen((char *)wiced_bt_cfg_settings.device_name);
			break;

		case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
			p_attribute_value_source    = &peripheral_gatt_generic_access_appearance;
			attribute_value_length      = sizeof(peripheral_gatt_generic_access_appearance);
			break;

		case HDLC_PROVISION_SSID_VALUE:
			p_attribute_value_source    = provision_ssid;
			attribute_value_length      = provision_ssid_len;
			break;

		case HDLC_PROVISION_SECURITY_VALUE:
			p_attribute_value_source    = &provision_security;
			attribute_value_length      = sizeof(provision_security);
			break;

		case HDLC_PROVISION_PASSWORD_VALUE:
			p_attribute_value_source    = provision_password;
			attribute_value_length      = provision_password_len;
			break;

		case HDLC_PROVISION_ACTION_VALUE:
			p_attribute_value_source    = &provision_action;
			attribute_value_length      = sizeof(provision_action);
			break;

		default:
			status = WICED_BT_GATT_READ_NOT_PERMIT;
			break;
    }

    /* Validate destination buffer size */
    if (attribute_value_length > *p_read_request->p_val_len)
    {
        *p_read_request->p_val_len = attribute_value_length;
    }

    /* Copy the attribute value */
    if (attribute_value_length)
    {
        memcpy(p_read_request->p_val, p_attribute_value_source, attribute_value_length);
    }

    /* Indicate number of bytes copied */
    *p_read_request->p_val_len = attribute_value_length;

    return (status);
}

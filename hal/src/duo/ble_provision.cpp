
#include <cstdlib>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "ble_provision_gatt_db.h"
#include "ble_provision.h"
#include "ble_hal.h"
#include "wlan_hal.h"
#include "rgbled.h"
#include "dct.h"
#include "deviceid_hal.h"
#include "flash_access.h"
#include "spark_macros.h"
#include "gpio_hal.h"
#include "core_hal.h"
#include "core_hal_stm32f2xx.h"
#include "hci_usart_hal.h"
#include "debug.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define BLE_TX_RX_STREAM_MAX              128
#define TX_RX_STREAM_CMD_IDX              0
#define TX_RX_STREAM_LEN_IDX              0

#define CONFIG_AP_ENTRY_LEN_MIN    (9)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum {
    GapStatus_Connected,
    GapStatus_Disconnect,
} GapStatus;

typedef enum {
    PROVISION_CMD_SCAN_REQUEST = 0xA0,
    PROVISION_CMD_CONFIG_AP_ENTRY,
    PROVISION_CMD_CONNECT_AP,
	PROVISION_CMD_NOTIFY_AP,
    PROVISION_CMD_NOTIFY_SYS_INFO,
	PROVISION_CMD_NOTIFY_IP_CONFIG,
} BLEProvisionCmd_t;

typedef enum {
    PROVISION_STA_IDLE = 0xB0,
    PROVISION_STA_SCANNING,
    PROVISION_STA_SCAN_COMPLETE,
    PROVISION_STA_CONFIG_AP,
    PROVISION_STA_CONNECTING,
    PROVISION_STA_CONNECTED,
    PROVISION_STA_CONNECT_FAILED,
} BLEProvisionSta_t;

typedef enum {
    CMD_PIPE_AVAILABLE,
    CMD_PIPE_BUSY_WRITE,
    CMD_PIPE_BUSY_NOTIFY,
} Cmd_pipe_state_t;

typedef enum {
    AP_CONFIGURED = 0xD0,
    AP_SCANNED,
} BLEProvisionAPState_t;

typedef struct {
    uint8_t                channel;
    wiced_security_t       security;
    uint8_t                SSID_len;
    uint8_t                SSID[SSID_NAME_SIZE];
    uint8_t                security_key_length;
    char                   security_key[ SECURITY_KEY_SIZE ];
} Config_ap_entry_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void ble_provision_init_variables(void);
static void ble_provision_notify(uint16_t attr_handle, uint8_t *pbuf, uint8_t len);
static void ble_provision_send_ap_details(wiced_scan_result_t* record);
static void ble_provision_send_ip_config(void);
static void ble_provision_send_sys_info(void);
static void ble_provision_parse_cmd(uint8_t *stream, uint8_t stream_len);

static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request);
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request);
static wiced_result_t ble_provision_scan_result_handler(wiced_scan_handler_result_t* malloced_scan_result);


/******************************************************
 *               Variable Definitions
 ******************************************************/
static uint8_t      ble_tx_rx_stream[BLE_TX_RX_STREAM_MAX];
static uint8_t      ble_tx_rx_stream_len = 0;
static uint8_t      rec_len = 0;

/* BLE connect variable */
static uint16_t 	connect_id = 0x0000;
static GapStatus  	connect_status = GapStatus_Disconnect;

/* GATT attrIbute values */
static uint32_t    	peripheral_gatt_attribute_service_changed = 0;
static uint16_t    	peripheral_gatt_generic_access_appearance = 0;
static uint8_t      command_value[20];
static uint8_t      command_value_len;
static uint16_t    	command_notify_flag = 0x0000;
static uint8_t		status_value;
static uint16_t		status_notify_flag = 0x0000;

static Cmd_pipe_state_t cmd_pipe_state = CMD_PIPE_AVAILABLE;

static wiced_bool_t device_configured = WICED_FALSE;
static uint8_t provision_status = PROVISION_STA_IDLE;

const unsigned USART6_Index = 87;

/* Stack and buffer pool configuration tables */
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

extern char link_interrupt_vectors_location;
extern char link_ram_interrupt_vectors_location;
extern char link_ram_interrupt_vectors_location_end;

#ifdef __cplusplus
extern "C" {
#endif

void bt_uart_irq(void);

wiced_result_t add_wiced_wifi_credentials(const char *ssid, uint16_t ssidLen, const char *password,
        uint16_t passwordLen, wiced_security_t security, unsigned channel);

#ifdef __cplusplus
}
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
/* Initialize peripheral */
void ble_provision_init(void) {
	static uint8_t init_done = 0;

    if(!init_done) {
        init_done = 1;

        HAL_Pin_Mode(BT_RTS, OUTPUT);
        HAL_GPIO_Write(BT_RTS, 1);
        wiced_rtos_delay_milliseconds(50);
        HAL_HCI_USART_registerReceiveHandler(NULL);
        HAL_HCI_USART_End(HAL_HCI_USART_SERIAL6);

        uint32_t* isrs = (uint32_t*)&link_ram_interrupt_vectors_location;
        isrs[USART6_Index] = (uint32_t)bt_uart_irq;

        HAL_Pin_Mode(BT_POWER, OUTPUT);
        HAL_GPIO_Write(BT_POWER, 0);
        wiced_rtos_delay_milliseconds(100);
        HAL_GPIO_Write(BT_POWER, 1);
        wiced_rtos_delay_milliseconds(100);

        bt_stack_init();

        wiced_bt_ble_advert_data_t adv_data;
        wiced_bt_ble_128service_t  service;

        uint8_t buf[16] = { UUID_SERVCLASS_BLE_PROVISION };

        /* Set advertising data: device name and discoverable flag */
        service.list_cmpl = WICED_TRUE;
        memcpy(service.uuid128, buf, MAX_UUID_SIZE);
        adv_data.flag = 0x06;
        adv_data.p_services_128b = &service;

        wiced_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_FLAGS|BTM_BLE_ADVERT_BIT_SERVICE_128|BTM_BLE_ADVERT_BIT_DEV_NAME, &adv_data);

        /* Enable privacy */
        wiced_bt_ble_enable_privacy( WICED_TRUE );

        /* Register for gatt event notifications */
        wiced_bt_gatt_register(&ble_provision_gatt_cback);

        /* Initialize GATT database */
        wiced_bt_gatt_db_init ((uint8_t *)gatt_db, gatt_db_size);
    }

    /* start LE advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    INFO("BLE Provisioning begin.\n");
}

void ble_provision_loop(void) {
    wiced_rtos_delay_milliseconds( 10 );
}

void ble_provision_on_failed(void) {
    provision_status = PROVISION_STA_CONNECT_FAILED;

    if(connect_id != 0x0000) {
        ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);
        wiced_rtos_delay_milliseconds(100);
        wiced_bt_gatt_disconnect(connect_id);
    }
    WARN("Connect to failed.\n");
}

void ble_provision_finalize(void) {
    provision_status = PROVISION_STA_CONNECTED;

    if(connect_id != 0x0000) {
        ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);
        wiced_rtos_delay_milliseconds(100);
        ble_provision_send_ip_config();
        wiced_bt_gatt_disconnect(connect_id);
    }

    wiced_rtos_delay_milliseconds( 500 );

    USB_Cable_Config(DISABLE);
    HAL_Core_System_Reset();
}

static void ble_provision_init_variables(void) {
    provision_status = PROVISION_STA_IDLE;
    device_configured = WICED_FALSE;
}

static void ble_provision_send_sys_info(void) {
    INFO("Send system info. \r\n");
    if((command_notify_flag == 0x0100) && (cmd_pipe_state == CMD_PIPE_AVAILABLE)) {
        uint16_t ver[4] = {0x0000, 0x0000, 0x0000, 0x0000};

        ver[0] = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08000000);
        ver[1] = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08020000);
        ver[2] = FLASH_ModuleVersion(FLASH_INTERNAL, 0x08040000);
        ver[3] = FLASH_ModuleVersion(FLASH_INTERNAL, 0x080C0000);

        ble_tx_rx_stream_len = 0;
        memset(ble_tx_rx_stream, '\0', BLE_TX_RX_STREAM_MAX);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0;                                                   // TX stream length
        ble_tx_rx_stream[ble_tx_rx_stream_len++] = PROVISION_CMD_NOTIFY_SYS_INFO;                       // Command

        HAL_device_ID(&ble_tx_rx_stream[ble_tx_rx_stream_len], 12);                                     // Device ID
        ble_tx_rx_stream_len += 12;

        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)ver, sizeof(ver));                   // Module versions
        ble_tx_rx_stream_len += sizeof(ver);

        const char *release_str = stringify(SYSTEM_VERSION_STRING);
        ble_tx_rx_stream[ble_tx_rx_stream_len++] = strlen(release_str);                                 // Release string length
        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], release_str, strlen(release_str));              // Release string
        ble_tx_rx_stream_len += strlen(release_str);

        ble_tx_rx_stream[TX_RX_STREAM_LEN_IDX] = ble_tx_rx_stream_len;
        ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, ble_tx_rx_stream, ble_tx_rx_stream_len);

        DEBUG_D("Send system info successfully.\r\n");
    }
}

static void ble_provision_send_ap_details(wiced_scan_result_t* record) {
    if((command_notify_flag == 0x0100) && (cmd_pipe_state == CMD_PIPE_AVAILABLE)) {
        ble_tx_rx_stream_len = 0;
        memset(ble_tx_rx_stream, '\0', BLE_TX_RX_STREAM_MAX);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0;                                                   // TX stream length
        ble_tx_rx_stream[ble_tx_rx_stream_len++] = (uint8_t)PROVISION_CMD_NOTIFY_AP;                    // Command

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = AP_SCANNED;                                          // AP state
        if(wlan_has_credentials() == 0) {
            platform_dct_wifi_config_t* wifi_config = NULL;
            wiced_result_t result = wiced_dct_read_lock( (void**) &wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(*wifi_config));
            if (result == WICED_SUCCESS) {
                if(!memcmp(wifi_config->stored_ap_list[0].details.BSSID.octet, record->BSSID.octet, 6))
                	ble_tx_rx_stream[ble_tx_rx_stream_len-1] = AP_CONFIGURED;
            }
            wiced_dct_read_unlock(wifi_config, WICED_FALSE);
        }

        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)&record->signal_strength, sizeof(int16_t));     // Signal strength
        ble_tx_rx_stream_len += sizeof(int16_t);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = record->channel;                                     // Channel

        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], record->BSSID.octet, sizeof(wiced_mac_t));      // BSSID
        ble_tx_rx_stream_len += sizeof(wiced_mac_t);

        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)&record->security, sizeof(wiced_security_t));   // Security type
        ble_tx_rx_stream_len += sizeof(wiced_security_t);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = record->SSID.length;                                 // SSID length
        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], record->SSID.value, record->SSID.length);       // SSID
        ble_tx_rx_stream_len += record->SSID.length;

        ble_tx_rx_stream[TX_RX_STREAM_LEN_IDX] = ble_tx_rx_stream_len;
        ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, ble_tx_rx_stream, ble_tx_rx_stream_len);
    }
}

static void ble_provision_send_ip_config(void) {
    if((command_notify_flag == 0x0100) && (cmd_pipe_state == CMD_PIPE_AVAILABLE)) {
        wiced_security_t   sec;
        wiced_bss_info_t   ap_info;
        wiced_ip_address_t station_ip, gateway_ip;
        wiced_mac_t        gateway_mac;

        ble_tx_rx_stream_len = 0;
        memset(ble_tx_rx_stream, '\0', BLE_TX_RX_STREAM_MAX);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0;                                                   // TX stream length
        ble_tx_rx_stream[ble_tx_rx_stream_len++] = (uint8_t)PROVISION_CMD_NOTIFY_IP_CONFIG;             // Command

        wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &station_ip );
        wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &gateway_ip );
        wwd_wifi_get_mac_address( &gateway_mac, WWD_AP_INTERFACE );
        wwd_wifi_get_ap_info( &ap_info, &sec );

        if(station_ip.version == WICED_IPV4) {                                                          // Device's IP
            ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0x04;
            memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)&station_ip.ip.v4, sizeof(uint32_t));
            ble_tx_rx_stream_len += sizeof(uint32_t);
        }
        else {
            ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0x06;
            memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)&station_ip.ip.v6, sizeof(station_ip.ip.v6));
            ble_tx_rx_stream_len += sizeof(station_ip.ip.v6);
        }

        if(gateway_ip.version == WICED_IPV4) {                                                          // Gateway's IP
            ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0x04;
            memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)&gateway_ip.ip.v4, sizeof(uint32_t));
            ble_tx_rx_stream_len += sizeof(uint32_t);
        }
        else {
            ble_tx_rx_stream[ble_tx_rx_stream_len++] = 0x06;
            memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], (uint8_t *)gateway_ip.ip.v6, sizeof(gateway_ip.ip.v6));
            ble_tx_rx_stream_len += sizeof(gateway_ip.ip.v6);
        }

        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], gateway_mac.octet, sizeof(wiced_mac_t));        // Gateway MAC address
        ble_tx_rx_stream_len += sizeof(wiced_mac_t);

        ble_tx_rx_stream[ble_tx_rx_stream_len++] = ap_info.SSID_len;                                    // Connected SSID length
        memcpy(&ble_tx_rx_stream[ble_tx_rx_stream_len], ap_info.SSID, ap_info.SSID_len);                // Connected SSID
        ble_tx_rx_stream_len += ap_info.SSID_len;

        ble_tx_rx_stream[TX_RX_STREAM_LEN_IDX] = ble_tx_rx_stream_len;
        ble_provision_notify(HDLC_BLE_PROVISION_COMMAND_VALUE, ble_tx_rx_stream, ble_tx_rx_stream_len);

        DEBUG_D("Send IP config.\r\n");

        wiced_rtos_delay_milliseconds(200);
    }
}

static void ble_provision_notify(uint16_t attr_handle, uint8_t *pbuf, uint8_t len) {

    if(attr_handle != HDLC_BLE_PROVISION_COMMAND_VALUE && attr_handle != HDLC_BLE_PROVISION_STATUS_VALUE)
        return;

    cmd_pipe_state = CMD_PIPE_BUSY_NOTIFY;

    if(connect_status == GapStatus_Connected) {
        uint8_t i=0, tx_len;
        while(i < len) {
            if(len-i >= 20)
                tx_len = 20;
            else
                tx_len = len - i;

            if(attr_handle == HDLC_BLE_PROVISION_COMMAND_VALUE) {
                memcpy(command_value, &pbuf[i], tx_len);
                command_value_len = tx_len;
            }
            else {
                status_value = *pbuf;
            }
            wiced_bt_gatt_send_notification(connect_id, attr_handle, tx_len, &pbuf[i]);
    	    i += tx_len;

            wiced_rtos_delay_milliseconds(20);
        }
	}
    cmd_pipe_state = CMD_PIPE_AVAILABLE;
}

static void ble_provision_parse_cmd(uint8_t *data, uint8_t data_len) {

	BLEProvisionCmd_t provision_cmd = (BLEProvisionCmd_t)data[0];

    DEBUG_D("BLE Provision command: %d\r\n", data[0]);

    switch(provision_cmd)
    {
        case PROVISION_CMD_NOTIFY_SYS_INFO:
            cmd_pipe_state = CMD_PIPE_AVAILABLE;
            if(provision_status != PROVISION_STA_SCANNING) {
                ble_provision_send_sys_info();
            }
            break;

        case PROVISION_CMD_SCAN_REQUEST:
            cmd_pipe_state = CMD_PIPE_AVAILABLE;
            if(provision_status != PROVISION_STA_SCANNING) {
                provision_status = PROVISION_STA_SCANNING;
                ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);
                wiced_wifi_scan_networks(ble_provision_scan_result_handler, NULL );
            }
            break;

        case PROVISION_CMD_CONFIG_AP_ENTRY:
            if((data_len-1) >= CONFIG_AP_ENTRY_LEN_MIN) {
                Config_ap_entry_t config_ap_entry;
                uint32_t security = 0x00000000;
                uint8_t i = 1; // The first byte is command

                config_ap_entry.channel = data[i++];                                                 // Channel
                security = (uint32_t)(data[i++]);
                security |= (uint32_t)(data[i++]<<8);
                security |= (uint32_t)(data[i++]<<16);
                security |= (uint32_t)(data[i++]<<24);
                config_ap_entry.security = (wiced_security_t)security;                               // Security
                config_ap_entry.SSID_len = data[i++];                                                // SSID length
                memcpy(config_ap_entry.SSID, &data[i], config_ap_entry.SSID_len);                    // SSID
                i += config_ap_entry.SSID_len;
                config_ap_entry.security_key_length = data[i++];                                     // Security key length
                memcpy(config_ap_entry.security_key, &data[i], config_ap_entry.security_key_length); // Security key

                add_wiced_wifi_credentials((const char *)config_ap_entry.SSID, config_ap_entry.SSID_len, \
                                           (const char *)config_ap_entry.security_key, config_ap_entry.security_key_length, \
                                           config_ap_entry.security, config_ap_entry.channel);

                device_configured = WICED_TRUE;

                provision_status = PROVISION_STA_CONFIG_AP;
                ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);

                DEBUG_D("WiFi credentials saved.\r\n");
            }
            cmd_pipe_state = CMD_PIPE_AVAILABLE;
            break;

        case PROVISION_CMD_CONNECT_AP:
            cmd_pipe_state = CMD_PIPE_AVAILABLE;
            if(device_configured == WICED_TRUE) {
                HAL_WLAN_notify_simple_config_done();

                provision_status = PROVISION_STA_CONNECTING;
                ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);

                DEBUG_D("Connecting to AP.\r\n");
            }
            break;

        default: break;
    }
}

/******************************************************
 *             Callback Function Definitions
 ******************************************************/
/* GATT event handler */
static wiced_bt_gatt_status_t ble_provision_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            /* GATT connection status change */
            if (p_event_data->connection_status.connected) {
                connect_status = GapStatus_Connected;
                connect_id = p_event_data->connection_status.conn_id;

                ble_provision_init_variables();

                /* Disable connectability. */
                wiced_bt_start_advertisements (BTM_BLE_ADVERT_OFF, 0, NULL);
            }
            else {
                connect_status = GapStatus_Disconnect;
                connect_id = 0x0000;
                command_notify_flag = 0x0000;
                status_notify_flag = 0x0000;

                /* Connection released. Re-enable BLE connectability. */
                if(device_configured != WICED_TRUE)
                    wiced_bt_start_advertisements (BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            /* GATT attribute read/write request */
            if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE)
                status = ble_provision_gatt_write_request(&p_event_data->attribute_request.data.write_req);
            else if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_READ)
                status = ble_provision_gatt_read_request(&p_event_data->attribute_request.data.read_req);
            break;

        default:
            break;
    }

    return (status);
}

/* Handler for attrubute write requests */
static wiced_bt_gatt_status_t ble_provision_gatt_write_request(wiced_bt_gatt_write_t *p_write_request) {
    uint8_t new_value[20];
    uint8_t new_value_len;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    new_value_len = p_write_request->val_len;
    memcpy(new_value, (uint8_t *)p_write_request->p_val, new_value_len);

    switch (p_write_request->handle) {
        /* Client Characteristic Configuration Descriptor */
        case HDLC_BLE_PROVISION_COMMAND_CCCD:
            if(new_value_len >= 2) {
                command_notify_flag = new_value[0];
                command_notify_flag = (command_notify_flag<<8) + new_value[1];
            }
            break;

        case HDLC_BLE_PROVISION_STATUS_CCCD:
            if(new_value_len >= 2) {
                status_notify_flag = new_value[0];
                status_notify_flag = (status_notify_flag<<8) + new_value[1];
            }
            break;

        /* Server characteristic attribute value */
        case HDLC_BLE_PROVISION_COMMAND_VALUE:
            if(cmd_pipe_state == CMD_PIPE_AVAILABLE) {
                ble_tx_rx_stream_len = new_value[0];
                if(ble_tx_rx_stream_len >= 2) {        // At least 2 byte: stream_len + command + [...]
                    rec_len = 0;
                    memset(ble_tx_rx_stream, '\0', BLE_TX_RX_STREAM_MAX);
                    cmd_pipe_state = CMD_PIPE_BUSY_WRITE;
                }
            }

            if(cmd_pipe_state == CMD_PIPE_BUSY_WRITE) {
                memcpy(&ble_tx_rx_stream[rec_len], new_value, new_value_len);
                rec_len += new_value_len;
                if(rec_len >= ble_tx_rx_stream_len) {
                	ble_provision_parse_cmd(&ble_tx_rx_stream[1], ble_tx_rx_stream_len-1);
                    ble_tx_rx_stream_len = 0;
                    cmd_pipe_state = CMD_PIPE_AVAILABLE;
                }
            }
            break;

        default:
            status = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
    }

    return (status);
}

/* Handler for attrubute read requests */
static wiced_bt_gatt_status_t ble_provision_gatt_read_request(wiced_bt_gatt_read_t *p_read_request) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    void *p_attribute_value_source;
    uint16_t attribute_value_length = 0;

    switch (p_read_request->handle) {
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

        case HDLC_BLE_PROVISION_COMMAND_CCCD:
            p_attribute_value_source    = &command_notify_flag;
            attribute_value_length      = sizeof(command_notify_flag);
            break;

        case HDLC_BLE_PROVISION_STATUS_CCCD:
            p_attribute_value_source    = &status_notify_flag;
            attribute_value_length      = sizeof(status_notify_flag);
            break;

        case HDLC_BLE_PROVISION_COMMAND_VALUE:
            p_attribute_value_source    = command_value;
            attribute_value_length      = command_value_len;
            break;

        case HDLC_BLE_PROVISION_STATUS_VALUE:
            p_attribute_value_source    = &status_value;
            attribute_value_length      = 1;
            break;

        default:
            status = WICED_BT_GATT_READ_NOT_PERMIT;
            break;
    }

    /* Validate destination buffer size */
    if (attribute_value_length > *p_read_request->p_val_len) {
        *p_read_request->p_val_len = attribute_value_length;
    }

    /* Copy the attribute value */
    if (attribute_value_length) {
        memcpy(p_read_request->p_val, p_attribute_value_source, attribute_value_length);
    }

    /* Indicate number of bytes copied */
    *p_read_request->p_val_len = attribute_value_length;

    return (status);
}

/* Callback function to handle scan results */
static wiced_result_t ble_provision_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result ) {
    malloc_transfer_to_curr_thread( malloced_scan_result );

    if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE ) {
        wiced_scan_result_t* record = &malloced_scan_result->ap_details;
        record->SSID.value[record->SSID.length] = 0; /* Ensure the SSID is null terminated */
        ble_provision_send_ap_details(record);
    }
    else {
        provision_status = PROVISION_STA_SCAN_COMPLETE;
        ble_provision_notify(HDLC_BLE_PROVISION_STATUS_VALUE, &provision_status, 1);
    }

    free( malloced_scan_result );

    return WICED_SUCCESS;
}

/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * Include Files
 */
#include "btstack.h"
#include "bt_control_bcm.h"
#include "btstack-config.h"
#include "hci_dump.h"
#include "run_loop_wiced.h"

#include "btstack_hal.h"
#include "wlan_hal.h"

typedef enum gattAction {
    gattActionWrite,
    gattActionSubscribe,
    gattActionUnsubscribe,
    gattActionServiceQuery,
    gattActionCharacteristicQuery,
    gattActionRead,
} gattAction_t;

/**
 * Local Variables
 */
/**@brief Configure of hci_usart. */
const hci_transport_config_uart_t hci_uart_config = {
    HCI_TRANSPORT_CONFIG_UART,
    HCI_INIT_BAUDRATE,
    HCI_MAIN_BAUDRATE,
    0,
    NULL
};

enum {
    SET_ADVERTISEMENT_PARAMS  = 1 << 0,
    SET_ADVERTISEMENT_DATA    = 1 << 1,
    SET_ADVERTISEMENT_ENABLED = 1 << 2,
};

/**@brief btstack state. */
static int btstack_state;
static uint8_t hci_init_flag = 0;

static uint16_t le_peripheral_todos = 0;

/**@brief connect timeout. */
static timer_source_t connection_timer;

/**@brief Gatt client. */
static uint16_t gatt_client_id;
static gattAction_t gattAction;

/**@brief BD address. */
static bool have_custom_addr;
static bd_addr_t public_bd_addr;

/**@brief Gatt read/write callback handler. */
static uint16_t (*gattReadCallback)(uint16_t handle, uint8_t * buffer, uint16_t buffer_size);
static int (*gattWriteCallback)(uint16_t handle, uint8_t *buffer, uint16_t buffer_size);

static void (*bleDeviceConnectedCallback)(BLEStatus_t status, uint16_t handle)= NULL;
static void (*bleDeviceDisconnectedCallback)(uint16_t handle) = NULL;

/**
 * Function Declare
 */
/**
 * @brief Hardware error handler.
 */
void bluetooth_hardware_error(){
    log_info("Bluetooth Hardware Error event. Restarting...\n\n\n");
    while(1);
}

/**@brief ATT Client Read Callback for Dynamic Data
 *
 * @Note  If buffer == NULL, don't copy data, just return size of value
 *        If buffer != NULL, copy data and return number bytes copied
 *
 * @param con_handle of hci le connection
 * @param attribute_handle to be written
 * @param offset into the value - used for queued writes and long attributes
 * @param buffer
 * @param buffer_size
 *
 * @returns 0 if write was ok, ATT_ERROR_PREPARE_QUEUE_FULL if no space in queue, ATT_ERROR_INVALID_OFFSET if offset is larger than max buffer
 */
static uint16_t att_read_callback(uint16_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    if(gattReadCallback) {
        return gattReadCallback(att_handle, buffer, buffer_size);
    }
    return 0;
}

/**@brief ATT Client Write Callback for Dynamic Data
 *
 * @param con_handle of hci le connection
 * @param attribute_handle to be written
 * @param transaction - ATT_TRANSACTION_MODE_NONE for regular writes, ATT_TRANSACTION_MODE_ACTIVE for prepared writes and ATT_TRANSACTION_MODE_EXECUTE
 * @param offset into the value - used for queued writes and long attributes
 * @param buffer
 * @param buffer_size
 *
 * @returns 0 if write was ok, ATT_ERROR_PREPARE_QUEUE_FULL if no space in queue, ATT_ERROR_INVALID_OFFSET if offset is larger than max buffer
 */
static int att_write_callback(uint16_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    if(gattWriteCallback){
        return gattWriteCallback(att_handle, buffer, buffer_size);
    }
    return 0;
}

/**@brief Packet handle.
 *
 * @param packet_type
 * @param channel
 * @param *packet
 * @param size
 */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){

    bd_addr_t addr;
    uint16_t handle;

    switch (packet_type) {

        case HCI_EVENT_PACKET:
            switch (packet[0]) {

                case BTSTACK_EVENT_STATE:
                    btstack_state = packet[2];
                    // bt stack activated, get started
                    if (packet[2] == HCI_STATE_WORKING) {
                        le_peripheral_todos |= SET_ADVERTISEMENT_PARAMS
                                            | SET_ADVERTISEMENT_DATA
                                            | SET_ADVERTISEMENT_ENABLED;
                    }
                    break;

                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    if (bleDeviceDisconnectedCallback) {
                        handle = READ_BT_16(packet, 3);
                        (*bleDeviceDisconnectedCallback)(handle);
                    }
                    le_peripheral_todos |= SET_ADVERTISEMENT_ENABLED;
                    break;

                case GAP_LE_ADVERTISING_REPORT: {

                    break;
                }

                case HCI_EVENT_COMMAND_COMPLETE:
                    if (COMMAND_COMPLETE_EVENT(packet, hci_read_bd_addr)) {
                        bt_flip_addr(addr, &packet[OFFSET_OF_DATA_IN_COMMAND_COMPLETE + 1]);
                        log_info("Local Address: %s\n", bd_addr_to_str(addr));
                        break;
                    }
                    break;

                case HCI_EVENT_LE_META:
                    switch (packet[2]) {
                        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                            handle = READ_BT_16(packet, 4);
                            log_info("Connection complete, handle 0x%04x\n", handle);
                            run_loop_remove_timer(&connection_timer);
                            if (!bleDeviceConnectedCallback)
                                break;
                            if (packet[3]){
                                (*bleDeviceConnectedCallback)(BLE_STATUS_CONNECTION_ERROR, 0x0000);
                            } else {
                                (*bleDeviceConnectedCallback)(BLE_STATUS_OK, handle);
                            }
                            break;
                        default:
                            break;
                    }
                    break;
            }
    }
    // if (client_packet_handler){
    //     (*client_packet_handler)(packet_type, channel, packet, size);
    // }
}


static void gatt_client_callback(uint8_t packet_type, uint8_t * packet, uint16_t size){

    // if (hci) event is not 4-byte aligned, event->handle causes crash
    // workaround: check event type, assuming GATT event types are contagious
    if (packet[0] < GATT_QUERY_COMPLETE) return;
    if (packet[0] > GATT_MTU) return;

//    uint16_t  con_handle = READ_BT_16(packet, 2);
//    uint8_t   status;
//    uint8_t * value;
//    uint16_t  value_handle;
//    uint16_t  value_length;

    switch(packet[0]){
        case GATT_SERVICE_QUERY_RESULT:

            break;
        case GATT_CHARACTERISTIC_QUERY_RESULT:

            break;
        case GATT_QUERY_COMPLETE:
            //status = READ_BT_16(packet, 4);
            switch (gattAction){
                case gattActionWrite:

                    break;
                case gattActionSubscribe:

                    break;
                case gattActionUnsubscribe:

                    break;
                case gattActionServiceQuery:

                    break;
                case gattActionCharacteristicQuery:

                    break;
                default:
                    break;
            };
            break;
        case GATT_NOTIFICATION:

            break;
        case GATT_INDICATION:

            break;
        case GATT_CHARACTERISTIC_VALUE_QUERY_RESULT:

            break;
        default:
            break;
    }
}


/**
 * @brief BTstack initialize.
 */
void hal_btstack_init(void)
{
    if(!hci_init_flag)
    {
        wlan_activate();

        have_custom_addr = false;
        // reset handler
        //bleAdvertismentCallback = NULL;
        bleDeviceConnectedCallback = NULL;
        bleDeviceDisconnectedCallback = NULL;
        //gattServiceDiscoveredCallback = NULL;
        //gattCharacteristicDiscoveredCallback = NULL;
        //gattCharacteristicNotificationCallback = NULL;

        att_db_util_init();

        btstack_memory_init();
        run_loop_init(run_loop_wiced_get_instance());

        hci_transport_t    * transport = hci_transport_h4_wiced_instance();
        bt_control_t       * control   = bt_control_bcm_instance();
        remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
        hci_init(transport, (void*)&hci_uart_config, control, remote_db);

        if (have_custom_addr){
            hci_set_bd_addr(public_bd_addr);
        }

        hci_set_hardware_error_callback(&bluetooth_hardware_error);

        l2cap_init();

        // setup central device db
        le_device_db_init();

        sm_init();

        att_server_init(att_db_util_get_address(),att_read_callback, att_write_callback);
        att_server_register_packet_handler(packet_handler);

        gatt_client_init();
        gatt_client_id = gatt_client_register_packet_handler(gatt_client_callback);

        // setup advertisements params
        uint16_t adv_int_min = 0x0030;
        uint16_t adv_int_max = 0x0030;
        uint8_t adv_type = 0;
        bd_addr_t null_addr;
        memset(null_addr, 0, 6);
        gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);

        // turn on!
        btstack_state = 0;
        hci_power_control(HCI_POWER_ON);

        // poll until working
        while (btstack_state != HCI_STATE_WORKING){
            run_loop_execute();
        }
        hci_init_flag = 1;
    }
}

void hal_btstack_deInit(void)
{
    if(hci_init_flag)
    {
        hci_close();
        run_loop_deInit();
    }
    hci_init_flag = 0;
}

/**
 * @brief BTstack loop.
 */
void hal_btstack_loop_execute(void)
{
    run_loop_execute();
}


/**
 * @brief Run loop set timer.
 *
 * @param[in]  *ts
 * @param[in]  timeout_in_ms
 */
void hal_btstack_setTimer(hal_timer_source_t *ts, uint32_t timeout_in_ms)
{
    run_loop_set_timer((timer_source_t *)ts, timeout_in_ms);
}

void hal_btstack_setTimerHandler(hal_timer_source_t *ts, void (*process)(void *_ts))
{
    run_loop_set_timer_handler((timer_source_t *)ts, process);
}

void hal_btstack_addTimer(hal_timer_source_t *timer)
{
    run_loop_add_timer((timer_source_t *)timer);
}

int hal_btstack_removeTimer(hal_timer_source_t *timer)
{
    return run_loop_remove_timer((timer_source_t *)timer);
}

uint32_t hal_btstack_getTimeMs(void)
{
    return run_loop_get_time_ms();
}

/**
 * @brief Open debugger.
 */
void hal_btstack_debugLogger(uint8_t flag)
{
    hci_dump_enable_log_level(LOG_LEVEL_INFO, flag);
}
void hal_btstack_debugError(uint8_t flag)
{
    hci_dump_enable_log_level(LOG_LEVEL_ERROR, flag);
}
void hal_btstack_enablePacketLogger(void)
{
    hci_dump_open(NULL, HCI_DUMP_STDOUT);
}

/***************************************************************
 * Gap API
***************************************************************/
/**
 * @brief Set public bd address.
 *
 * @param[in]  public_bd_addr
 */
void hal_btstack_setPublicBdAddr(addr_t public_bd_addr)
{
    have_custom_addr = true;
    memcpy(public_bd_addr, public_bd_addr ,6);
}

/**
 * @brief Set local name.
 *
 * @param[in]  *local_name  Name string.
 */
void hal_btstack_setLocalName(const char *local_name)
{
    gap_set_local_name(local_name);
}

/**
 * @brief Set advertising data.
 *
 * @note
 *
 * @param[in]  size  The size of advertising data, no more than 31bytes.
 * @param[in]  data  Advertising data.
 *
 */
void hal_btstack_setAdvData(uint16_t size, uint8_t *data)
{
    gap_advertisements_set_data(size, data);
}

/**
 * @brief Set Advertisement Paramters.
 *
 * @note
 *
 * @param[in]  adv_int_min
 * @param[in]  adv_int_max
 * @param[in]  adv_type
 * @param[in]  dir_addr_type
 * @param[in]  dir_addr
 * @param[in]  channel_map
 * @param[in]  filter_policy
 *
 */
void hal_btstack_setAdvParams(uint16_t adv_int_min, uint16_t adv_int_max, uint8_t adv_type, uint8_t dir_addr_type, addr_t dir_addr, uint8_t channel_map, uint8_t filter_policy)
{
    gap_advertisements_set_params(adv_int_min,adv_int_max,adv_type,dir_addr_type,dir_addr,channel_map,filter_policy);
}

/**
 * @brief Start advertising.
 */
void hal_btstack_startAdvertising(void)
{
    gap_advertisements_enable(1);
}

/**
 * @brief Stop advertising.
 */
void hal_btstack_stopAdvertising(void)
{
    gap_advertisements_enable(0);
}

/**
 * @brief Set connected Callback.
 */
void hal_btstack_setConnectedCallback(void (*callback)(BLEStatus_t status, uint16_t handle))
{
    bleDeviceConnectedCallback = callback;
}

/**
 * @brief Set disconnected Callback.
 */
void hal_btstack_setDisconnectedCallback(void (*callback)(uint16_t handle))
{
    bleDeviceDisconnectedCallback = callback;
}

/**
 * @brief Disconnect by peripheral.
 */
static timer_source_t disconnect_time;
static uint16_t disconnect_handle = 0xFFFF;

static void disconnect_task(struct timer *ts)
{
    if(disconnect_handle != 0xFFFF)
        gap_disconnect(disconnect_handle);
    disconnect_handle = 0xFFFF;
}

void hal_btstack_disconnect(uint16_t handle)
{
    disconnect_handle = handle;
    disconnect_time.process = &disconnect_task;
    run_loop_set_timer(&disconnect_time, 20);
    run_loop_add_timer(&disconnect_time);
}

/***************************************************************
 * Gatt server API
***************************************************************/
/**
 * @brief Check if a notification or indication can be send right now
 *
 * @return 1, if packet can be sent
 */
int hal_btstack_attServerCanSend(void)
{
    return att_server_can_send();
}

/**
 * @brief Send notify to client.
 *
 * @note
 *
 * @param[in]  value_handle
 * @param[in]  *value
 * @param[in]  length
 *
 * @return 0 Success.
 */
int hal_btstack_attServerSendNotify(uint16_t value_handle, uint8_t *value, uint16_t length)
{
    return att_server_notify(value_handle, value, length);
}

/**
 * @brief Send indicate to client.
 *
 * @param[in]  value_handle
 * @param[in]  *value
 * @param[in]  length
 *
 * @return 0 Success.
 */
int hal_btstack_attServerSendIndicate(uint16_t value_handle, uint8_t *value, uint16_t length)
{
    return att_server_indicate(value_handle, value, length);
}

/**
 * @brief Set read callback of gatt characteristic.
 *
 * @param[in]  cb
 */
void hal_btstack_setGattCharsRead(uint16_t (*cb)(uint16_t handle, uint8_t *buffer, uint16_t buffer_size))
{
    gattReadCallback = cb;
}

/**
 * @brief Set write callback of gatt characteristic.
 *
 * @param[in]  cb
 */
void hal_btstack_setGattCharsWrite(int (*cb)(uint16_t handle, uint8_t *buffer, uint16_t buffer_size))
{
    gattWriteCallback = cb;
}

/**
 * @brief Add a 16bits-UUID service.
 *
 * @param[in]  uuid
 *
 */
void hal_btstack_addServiceUUID16bits(uint16_t uuid)
{
    att_db_util_add_service_uuid16(uuid);
}

/**
 * @brief Add a 128bits-UUID service.
 *
 * @param[in]  *uuid  Buffer of 128bits-UUID.
 *
 */
void hal_btstack_addServiceUUID128bits(uint8_t *uuid)
{
    att_db_util_add_service_uuid128(uuid);
}

/**
 * @brief Add a 16bits-UUID characteristic.
 *
 * @param[in]  uuid
 *
 * @return ::Attribute handle.
 */
uint16_t hal_btstack_addCharsUUID16bits(uint16_t uuid, uint16_t flags, uint8_t *data, uint16_t data_len)
{
    return att_db_util_add_characteristic_uuid16(uuid, flags, data, data_len);
}

uint16_t hal_btstack_addCharsDynamicUUID16bits(uint16_t uuid, uint16_t flags, uint8_t *data, uint16_t data_len)
{
    return att_db_util_add_characteristic_uuid16(uuid, flags|PROPERTY_DYNAMIC, data, data_len);
}


/**
 * @brief Add a 128bits-UUID characteristic.
 *
 * @param[in]  *uuid  Buffer of 128bits-UUID.
 *
 * @return ::Attribute handle.
 */
uint16_t hal_btstack_addCharsUUID128bits(uint8_t *uuid, uint16_t flags, uint8_t *data, uint16_t data_len)
{
    return att_db_util_add_characteristic_uuid128(uuid, flags, data, data_len);
}

uint16_t hal_btstack_addCharsDynamicUUID128bits(uint8_t *uuid, uint16_t flags, uint8_t *data, uint16_t data_len)
{
    return att_db_util_add_characteristic_uuid128(uuid, flags|PROPERTY_DYNAMIC, data, data_len);
}

/***************************************************************
 * Gatt client API
***************************************************************/
/**
 * @brief Start scanning.
 */
void hal_btstack_startScanning(void)
{
    le_central_start_scan();
}

/**
 * @brief Stop scanning.
 */
void hal_btstack_stopScanning(void)
{
    le_central_stop_scan();
}


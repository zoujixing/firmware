

/**
 * Include Files
 */
#include "wiced.h"
#include "btstack.h"
#include "btstack_chipset_bcm.h"
#include "btstack_config.h"
#include "hci_dump.h"
#include "btstack_run_loop_wiced.h"

#include "btstack_hal.h"
#include "wlan_hal.h"
#include "usb_hal.h"

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

/**@brief notify/indicate data node */
typedef struct{
    uint16_t handle;
    uint8_t  data_flag_len; //7-bit:1 indicate,0 notify. 6~0-bit:length of data.
    uint8_t  data[20];
}hal_notifyData_t;

/**@brief Queue for notify/indicate */
typedef struct{
    hal_notifyData_t queue[MAX_NO_NOTIFY_DATA_QUEUE];
    uint8_t head;
    uint8_t tail;
}hal_notifyDataQueue_t;

static hal_notifyDataQueue_t notify_queue={.head=0,.tail=0};

/**@brief btstack state. */
static int btstack_state;
static uint8_t hci_init_flag = 0;

static uint16_t le_peripheral_todos = 0;

/**@brief connect timeout. */
static btstack_timer_source_t connection_timer;

/**@brief Gatt client. */
static uint16_t gatt_client_id;
static gattAction_t gattAction;

/**@brief BD address. */
static bool have_custom_addr;
static bd_addr_t public_bd_addr;


static wiced_thread_t hal_btstack_thread_;
static uint8_t hal_btstack_thread_quit=1;

static void hal_stack_thread(uint32_t arg);

/**@brief Gatt read/write callback handler. */
static uint16_t (*gattReadCallback)(uint16_t handle, uint8_t * buffer, uint16_t buffer_size);
static int (*gattWriteCallback)(uint16_t handle, uint8_t *buffer, uint16_t buffer_size);

static void (*bleAdvertismentCallback)(advertisementReport_t * bleAdvertisement) = NULL;
static void (*bleDeviceConnectedCallback)(BLEStatus_t status, uint16_t handle)= NULL;
static void (*bleDeviceDisconnectedCallback)(uint16_t handle) = NULL;

static uint8_t notify_queueFreeSize(void);
static uint8_t notify_queneUsedSize(void);
static uint8_t notify_queneWrite(hal_notifyData_t *dat);
static uint8_t notify_queneRead(hal_notifyData_t *dat);

/**
 * Function Declare
 */
/**
 * @brief Thread for BLE loop.
 */
void hal_stack_thread(uint32_t arg) {
    while(!hal_btstack_thread_quit)
    {
        hal_btstack_loop_execute();
    }
    WICED_END_OF_CURRENT_THREAD( );
}

/**
 * @brief Hardware error handler.
 */
void paseAdvertisemetReport(advertisementReport_t *report, uint8_t *data)
{
    report->advEventType = data[2];
    report->peerAddrType = data[3];
    report->rssi         = data[10]-256;
    report->advDataLen   = data[11];
    memcpy(report->advData, &data[12], report->advDataLen);
    bt_flip_addr(report->peerAddr, &data[4]);
}

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

                case GAP_LE_ADVERTISING_REPORT:
                    if(bleAdvertismentCallback) {
                        advertisementReport_t report;
                        paseAdvertisemetReport(&report, packet);
                        (*bleAdvertismentCallback)(&report);
                    }
                    break;

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
                            btstack_run_loop_remove_timer(&connection_timer);
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

        // reset handler
        bleAdvertismentCallback = NULL;
        bleDeviceConnectedCallback = NULL;
        bleDeviceDisconnectedCallback = NULL;
        //gattServiceDiscoveredCallback = NULL;
        //gattCharacteristicDiscoveredCallback = NULL;
        //gattCharacteristicNotificationCallback = NULL;

        att_db_util_init();

        btstack_memory_init();
        btstack_run_loop_init(btstack_run_loop_wiced_get_instance());

        const hci_transport_t   * transport = hci_transport_h4_instance();
        remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
        hci_init(transport, (void*)&hci_uart_config, remote_db);

        hci_set_chipset(btstack_chipset_bcm_instance());

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

        memset(&notify_queue, 0x00, sizeof(hal_notifyDataQueue_t));

        // turn on!
        btstack_state = 0;
        hci_power_control(HCI_POWER_ON);

        // poll until working
        while (btstack_state != HCI_STATE_WORKING){
            btstack_run_loop_execute();
        }
        hal_btstack_thread_quit = 0;
        wiced_rtos_create_thread(&hal_btstack_thread_, WICED_APPLICATION_PRIORITY, "BLE provision", hal_stack_thread, 1024*3, NULL);
        hci_init_flag = 1;
    }
}

void hal_btstack_deInit(void)
{
    if(hci_init_flag)
    {
        have_custom_addr = false;
        hci_close();
        btstack_run_loop_deInit();
        hal_btstack_thread_quit = 1;
    }
    hci_init_flag = 0;
}

/**
 * @brief BTstack loop.
 */
void hal_btstack_loop_execute(void)
{
    if(hci_init_flag)
    {
        if(notify_queneUsedSize() && att_server_can_send_packet_now())
        {
            hal_notifyData_t data;
            notify_queneRead(&data);
            if((data.data_flag_len & 0x80) == 0x00)
                att_server_notify(data.handle, data.data, data.data_flag_len&0x7F);
            else
                att_server_indicate(data.handle, data.data, data.data_flag_len&0x7F);
        }
        btstack_run_loop_execute();
    }
}


/**
 * @brief Run loop set timer.
 *
 * @param[in]  *ts
 * @param[in]  timeout_in_ms
 */
void hal_btstack_setTimer(hal_timer_source_t *ts, uint32_t timeout_in_ms)
{
    btstack_run_loop_set_timer((btstack_timer_source_t *)ts, timeout_in_ms);
}

void hal_btstack_setTimerHandler(hal_timer_source_t *ts, void (*process)(void *_ts))
{
    btstack_run_loop_set_timer_handler((btstack_timer_source_t *)ts, process);
}

void hal_btstack_addTimer(hal_timer_source_t *timer)
{
    btstack_run_loop_add_timer((btstack_timer_source_t *)timer);
}

int hal_btstack_removeTimer(hal_timer_source_t *timer)
{
    return btstack_run_loop_remove_timer((btstack_timer_source_t *)timer);
}

uint32_t hal_btstack_getTimeMs(void)
{
    return btstack_run_loop_get_time_ms();
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


void hal_btstack_getAdvertisementAddr(uint8_t *addr_type, addr_t addr)
{
    hci_le_advertisement_address(addr_type, addr);
}

/**
 * @brief Set mode of random address.
 *
 * @param[in]  random_address_type
 */
void hal_btstack_setRandomAddressMode(uint8_t random_address_type)
{
    gap_random_address_set_mode((gap_random_address_type_t)random_address_type);
}

/**
 * @brief Set random address.
 *
 * @param[in]  addr
 */
void hal_btstack_setRandomAddr(addr_t addr)
{
    gap_random_address_set(addr);
}

/**
 * @brief Set public bd address.
 *
 * @param[in]  public_bd_addr
 */
void hal_btstack_setPublicBdAddr(addr_t addr)
{
    have_custom_addr = true;
    memcpy(public_bd_addr, addr ,6);
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
static btstack_timer_source_t disconnect_time;
static uint16_t disconnect_handle = 0xFFFF;

static void disconnect_task(struct btstack_timer_source *ts)
{
    if(disconnect_handle != 0xFFFF)
        gap_disconnect(disconnect_handle);
    disconnect_handle = 0xFFFF;
    memset(&notify_queue, 0x00, sizeof(hal_notifyDataQueue_t));
}

void hal_btstack_disconnect(uint16_t handle)
{
    disconnect_handle = handle;
    disconnect_time.process = &disconnect_task;
    btstack_run_loop_set_timer(&disconnect_time, 20);
    btstack_run_loop_add_timer(&disconnect_time);
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
    if(!notify_queueFreeSize())
        return 0;
        
    return 1;
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
    hal_notifyData_t data;
    log_info("send notify \r\n");
    data.handle        = value_handle;
    data.data_flag_len = length;
    memset(data.data, 0x00, 20);
    memcpy(data.data, value,  length);

    return notify_queneWrite(&data);
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
    hal_notifyData_t data;

    data.handle        = value_handle;
    data.data_flag_len = length | 0x80;
    memset(data.data, 0x00, 20);
    memcpy(data.data, value,  length);

    return notify_queneWrite(&data);
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

/**
 * @brief Set advertisement report callback for scanning device.
 */
void hal_btstack_setBLEAdvertisementCallback(void (*cb)(advertisementReport_t *advertisement_report))
{
    bleAdvertismentCallback = cb;
}

/***************************************************************
 * Other API
***************************************************************/
/**
 * @brief check whether has available space.
 *
 * @return 0    : full
 * 	       else : number of free.
 */
static uint8_t notify_queueFreeSize(void)
{
    return ( MAX_NO_NOTIFY_DATA_QUEUE - 1 - ((MAX_NO_NOTIFY_DATA_QUEUE + notify_queue.head - notify_queue.tail) % MAX_NO_NOTIFY_DATA_QUEUE));
}

/**
 * @brief Check whether has data to be sent.
 *
 * @return 0    : empty
 *         else : number of data.
 */
static uint8_t notify_queneUsedSize(void)
{
    return ((MAX_NO_NOTIFY_DATA_QUEUE + notify_queue.head - notify_queue.tail) % MAX_NO_NOTIFY_DATA_QUEUE);
}

/**
 * @brief Put a notify data to queue.
 *
 * @return 0:FAIL.
 *         1:SUCCESS.
 */
static uint8_t notify_queneWrite(hal_notifyData_t *dat)
{
    if(!notify_queueFreeSize())
        return 0;

    notify_queue.queue[notify_queue.head] = *dat;
    notify_queue.head = (notify_queue.head + 1) % MAX_NO_NOTIFY_DATA_QUEUE;

    return 1;
}

/**
 * @brief Read a notify data from queue.
 *
 * @return 0:FAIL.
 *         1:SUCCESS.
 */
static uint8_t notify_queneRead(hal_notifyData_t *dat)
{
    if(!notify_queneUsedSize())
        return 0;

    *dat = notify_queue.queue[notify_queue.tail];
    notify_queue.tail = (notify_queue.tail + 1) % MAX_NO_NOTIFY_DATA_QUEUE;

    return 1;
}

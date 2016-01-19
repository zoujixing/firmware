#include <cstdlib>
#include "wiced.h"
#include "device_name.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool fetch_or_generate_setup_ssid(wiced_ssid_t* SSID);

void HAL_Device_Name(device_name_t *dev_name)
{
    fetch_or_generate_setup_ssid( (wiced_ssid_t *)dev_name );
}

#ifdef __cplusplus
}
#endif


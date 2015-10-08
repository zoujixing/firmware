#define DYNALIB_IMPORT
#include "wifi_dynalib.h"
#include "spi_flash.h"
#include "flash_mal.h"

void wwd_load_firmware_image(uint8_t* buffer, uint32_t offset, uint32_t size)
{
	uint32_t address = EXTERNAL_FLASH_WIFI_FIRMWARE_ADDRESS + offset;

	sFLASH_ReadBuffer(buffer, address, size);
}

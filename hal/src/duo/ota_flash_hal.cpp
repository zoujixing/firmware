/**
 ******************************************************************************
 * @file    ota_flash_hal.cpp
 * @author  Matthew McGowan, Satish Nair
 * @version V1.0.0
 * @date    25-Sept-2014
 * @brief
 ******************************************************************************
  Copyright (c) 2013-2015 Particle Industries, Inc.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#include "ota_module.h"
#include "spark_macros.h"
#include "ota_flash_hal_stm32f2xx.h"
#include "core_hal.h"
#include "dct.h"

#if MODULAR_FIRMWARE
const module_bounds_t module_bootloader = { 0x4000, 0x8000000, 0x8004000, MODULE_FUNCTION_BOOTLOADER, 0, MODULE_STORE_MAIN };
const module_bounds_t module_system_part1 = { 0x20000, 0x8020000, 0x8040000, MODULE_FUNCTION_SYSTEM_PART, 1, MODULE_STORE_MAIN };
const module_bounds_t module_system_part2 = { 0x80000, 0x8040000, 0x80C0000, MODULE_FUNCTION_SYSTEM_PART, 2, MODULE_STORE_MAIN};
const module_bounds_t module_user = { 0x40000, 0x80C0000, 0x8100000, MODULE_FUNCTION_USER_PART, 1, MODULE_STORE_MAIN};
const module_bounds_t module_factory = { 0x40000, 0x140000, 0x180000, MODULE_FUNCTION_USER_PART, 1, MODULE_STORE_FACTORY};
const module_bounds_t* module_bounds[] = { &module_bootloader, &module_system_part1, &module_system_part2, &module_user, &module_factory };

const module_bounds_t module_ota = { 0x80000, 0xC0000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section1 = { 0x70000, 0xD0000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section2 = { 0x60000, 0xE0000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section3 = { 0x50000, 0xF0000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section4 = { 0x40000, 0x100000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section5 = { 0x30000, 0x110000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section6 = { 0x20000, 0x120000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t module_ota_section7 = { 0x10000, 0x130000, 0x140000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
const module_bounds_t* module_ota_bounds[] = { &module_ota_section1, &module_ota_section2, &module_ota_section3, &module_ota_section4, &module_ota_section5, &module_ota_section6, &module_ota_section7};
#else
const module_bounds_t module_bootloader = { 0x4000, 0x8000000, 0x8004000, MODULE_FUNCTION_BOOTLOADER, 0, MODULE_STORE_MAIN};
const module_bounds_t module_user = { 0x60000, 0x8020000, 0x8080000, MODULE_FUNCTION_MONO_FIRMWARE, 0, MODULE_STORE_MAIN};
const module_bounds_t module_factory = { 0x60000, 0x8080000, 0x80E0000, MODULE_FUNCTION_MONO_FIRMWARE, 0, MODULE_STORE_FACTORY};
const module_bounds_t* module_bounds[] = { &module_bootloader, &module_user, &module_factory };

const module_bounds_t module_ota = { 0x60000, 0x8080000, 0x80E0000, MODULE_FUNCTION_NONE, 0, MODULE_STORE_SCRATCHPAD};
#endif

const unsigned module_bounds_length = arraySize(module_bounds);

void HAL_OTA_Add_System_Info(hal_system_info_t* info, bool create, void* reserved)
{
    // presently no additional key/value pairs to send back
    info->key_values = NULL;
    info->key_value_count = 0;
}


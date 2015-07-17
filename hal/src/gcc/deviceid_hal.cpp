/**
 ******************************************************************************
 * @file    deviceid_hal.cpp
 * @author  Matthew McGowan
 * @version V1.0.0
 * @date    27-Sept-2014
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


#include "deviceid_hal.h"
#include "device_config.h"
#include "filesystem.h"

#include <stddef.h>
#include <algorithm>
#include <string>
#include <string.h>
#include <cstdio>
#include <limits>



using std::string;

uint8_t hex2dec(char c) {
    if (c<='9')
        return uint8_t(c-'0');
    if (c<='Z')
        return uint8_t(c-'A');
    return uint8_t(c-'a');
}


unsigned HAL_device_ID(uint8_t* dest, unsigned destLen)
{
    string device_id = get_configuration_value(DEVICE_ID, "000000000000000000000000");
    uint8_t len = device_id.length()>>1;
    if (dest && destLen<len)
        len = destLen;
    if (dest) {
        for (int i=0; i<len; i++) {
            char c1 = device_id[i*2];
            char c2 = device_id[i*2+1];
            uint8_t b = hex2dec(c1) << 4 | hex2dec(c2);
            dest[i] = b;
        }
    }
    return len;
}

unsigned HAL_Platform_ID()
{
    return PLATFORM_ID;
}
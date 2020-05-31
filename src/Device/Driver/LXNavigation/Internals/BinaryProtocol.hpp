/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_NMEAV1_PROTOCOL_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_NMEAV1_PROTOCOL_HPP

#include <cstdint>

namespace LXNavigation
{
namespace Binary
{
enum class Codes
{
  STX = 0x02,
  ACK = 0x06,
  NACK = 0x15,
};

enum class Commands
{
  SYN = 0x16,
  GET_LOGGER_INFO = 0xC4,
  SET_TASK = 0xCA,
  GET_TASK = 0xCB,
  SET_CLASS = 0xD0,
  GET_FLIGHT_INFO = 0xF0,
  GET_FLIGHT_BLOCK = 0xF1,
  GET_NUMBER_FLIGHTS = 0xF2,
  SEND_TO_RADIO = 0xF3,
  SET_OBS_ZONE = 0xF4,
  GET_OBS_ZONE = 0xF5
};

void GetFlightBlock(uint16_t flight_id, uint16_t flight_block, uint8_t* block_data);
}
}
#endif

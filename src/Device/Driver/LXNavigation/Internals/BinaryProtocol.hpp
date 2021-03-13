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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_BINARY_PROTOCOL_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_BINARY_PROTOCOL_HPP

#include "LXNavigationData.hpp"

#include <cstdint>
#include <boost/container/static_vector.hpp>

namespace LXNavigation
{
namespace Binary
{
constexpr int max_packet_size = 500;
using PacketBuffer = boost::container::static_vector<uint8_t, max_packet_size>;

constexpr int max_flight_block_size = 8192;
using FlightBlockBuffer = boost::container::static_vector<uint8_t, max_flight_block_size>;

bool ParseResult(const PacketBuffer& data);

void GenerateLoggerInfoGet(PacketBuffer& data);
std::optional<DeviceInfo> ParseLoggerInfo(const PacketBuffer& data);

void GenerateClass(PacketBuffer& data);

void GenerateTaskGet(PacketBuffer& data);
void GenerateTask(const TurnpointDataContainer& device_info, PacketBuffer& data);
std::optional<TurnpointDataContainer> ParseTask(const PacketBuffer& data);

void GenerateFlightNumberGet(PacketBuffer& data);
std::optional<uint16_t> ParseFlightNumber(const PacketBuffer& data);

void GenerateFlightInfoGet(uint16_t flight_id, PacketBuffer& data);
std::optional<FlightInfo> ParseFlightInfo(const PacketBuffer& data);

void GenerateFlightBlockGet(uint16_t flight_id, uint16_t flight_block, PacketBuffer& data);
void ParseFlightBlock(const FlightBlockBuffer& data, FlightBlockBuffer& block_data);

void GenerateTurnpointZoneGet(PacketBuffer& data);
void GenerateTurnpointZone(const TurnpointZone& zone, PacketBuffer& data);
std::optional<TurnpointZone> ParseTurnpointZone(const PacketBuffer& data);

void GenerateSendToRadio(PacketBuffer& data, const PacketBuffer& radio_data);

}
}
#endif

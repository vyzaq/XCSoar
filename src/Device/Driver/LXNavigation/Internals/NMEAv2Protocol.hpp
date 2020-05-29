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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_LXDT_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_LXDT_HPP

#include "Util/StaticString.hxx"
#include "Device/Driver/LXNavigation/Internals/LXNavigationData.hpp"

class NMEAInputLine;

namespace LXNavigation
{
namespace NMEAv2
{
using Message = NarrowString<60>;

enum class SentenceAction
{
  INFO,
  TP,
  ZONE,
  GLIDER,
  PILOT,
  TSK_PAR,
  MC_BAL,
  RADIO,
  R_SWITCH,
  R_DUAL,
  R_SPACING,
  FLIGHTS_NO,
  FLIGHT_INFO,
  ERROR,
  OK
};

enum class SentenceCode
{
  GET,
  SET,
  ANS
};

bool IsLineMatch(const NMEAInputLine& nmea_line, Sentences sentence, SentenceAction command, SentenceCode status);

StatusResult ParseLXDT_ANS_Status(const NMEAInputLine &line);

Message GenerateLXDT_INFO_GET();
DeviceInfo ParseLXDT_INFO_ANS(const NMEAInputLine &line);

Message GenerateLXDT_TP_GET(u_int16_t turnpoint_id);
Message GenerateLXDT_TP_SET(const TurnpointData& data);
TurnpointData ParseLXDT_TP_ANS(const NMEAInputLine &line);

Message GenerateLXDT_ZONE_GET(u_int16_t turnpoint_id);
Message GenerateLXDT_ZONE_SET(const TurnpointZone& data);
TurnpointZone ParseLXDT_ZONE_ANS(const NMEAInputLine &line);

Message GenerateLXDT_GLIDER_GET();
Message GenerateLXDT_GLIDER_SET(const GliderInfo& data);
GliderInfo ParseLXDT_GLIDER_ANS(const NMEAInputLine &line);

Message GenerateLXDT_PILOT_GET();
Message GenerateLXDT_PILOT_SET(const PilotInfo& data);
PilotInfo ParseLXDT_PILOT_ANS(const NMEAInputLine &line);

Message GenerateLXDT_TSK_PAR_GET();
Message GenerateLXDT_TSK_PAR_SET(const TaskParameters& data);
TaskParameters ParseLXDT_TSK_PAR_ANS(const NMEAInputLine &line);

Message GenerateLXDT_MC_BAL_GET();
Message GenerateLXDT_MC_BAL_SET(const std::pair<GlideParameters, DeviceParameters>& data);
std::pair<GlideParameters, DeviceParameters> ParseLXDT_MC_BAL_ANS(const NMEAInputLine &line);

Message GenerateLXDT_RADIO_GET();
Message GenerateLXDT_RADIO_SET(const RadioParameters& data);
RadioParameters ParseLXDT_RADIO_ANS(const NMEAInputLine &line);

Message GenerateLXDT_FLIGHTS_NO_GET();
int16_t ParseLXDT_FLIGHTS_NO_ANS(const NMEAInputLine &line);

Message GenerateLXDT_FLIGHT_INFO_GET(u_int16_t turnpoint_id);
FlightInfo ParseLXDT_FLIGHT_INFO_ANS(const NMEAInputLine &line);

Message GenerateLXDT_R_SWITCH_TOGGLE();
Message GenerateLXDT_R_DUAL_SET(bool is_dual_enabled);
Message GenerateLXDT_R_SPACING_SET(bool is_833);
}
}
#endif

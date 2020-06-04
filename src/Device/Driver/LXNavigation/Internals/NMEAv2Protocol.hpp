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

#include "Device/Driver/LXNavigation/Internals/LXNavigationData.hpp"
#include "ProtocolDefinitions.hpp"
#include "ProtocolMatchers.hpp"

#include "NMEA/InputLine.hpp"

class NMEAInputLine;
struct NMEAInfo;

namespace LXNavigation
{
namespace NMEAv2
{
template<Sentences sentence, SentenceCode command, SentenceAction status>
bool IsLineMatch(const NMEAInputLine& nmea_line)
{
  NMEAInputLine line(nmea_line);
  return MatchSentence<sentence>(line) && MatchStatus<status>(line) && MatchCommand<command>(line);
}

Message ParseLXDT_ANS_Status(NMEAInputLine &line);

Message GenerateLXDT_INFO_GET();
DeviceInfo ParseLXDT_INFO_ANS(NMEAInputLine &line);

Message GenerateLXDT_TP_GET(u_int16_t turnpoint_id);
Message GenerateLXDT_TP_SET(const TurnpointData& data);
std::optional<TurnpointData> ParseLXDT_TP_ANS(NMEAInputLine &line);

Message GenerateLXDT_ZONE_GET(u_int16_t turnpoint_id);
Message GenerateLXDT_ZONE_SET(const TurnpointZone& data);
std::optional<TurnpointZone> ParseLXDT_ZONE_ANS(NMEAInputLine &line);

Message GenerateLXDT_GLIDER_GET();
Message GenerateLXDT_GLIDER_SET(const GliderInfo& data);
GliderInfo ParseLXDT_GLIDER_ANS(NMEAInputLine &line);

Message GenerateLXDT_PILOT_GET();
Message GenerateLXDT_PILOT_SET(const PilotInfo& data);
PilotInfo ParseLXDT_PILOT_ANS(NMEAInputLine &line);

Message GenerateLXDT_TSK_PAR_GET();
Message GenerateLXDT_TSK_PAR_SET(const TaskParameters& data);
std::optional<TaskParameters> ParseLXDT_TSK_PAR_ANS(NMEAInputLine &line);

Message GenerateLXDT_MC_BAL_GET();
Message GenerateLXDT_MC_BAL_SET(const std::pair<GlideParameters, DeviceParameters>& data);
std::pair<GlideParameters, DeviceParameters> ParseLXDT_MC_BAL_ANS(NMEAInputLine &line);

Message GenerateLXDT_RADIO_GET();
Message GenerateLXDT_RADIO_SET(const RadioParameters& data);
std::optional<RadioParameters> ParseLXDT_RADIO_ANS(NMEAInputLine &line);

Message GenerateLXDT_FLIGHTS_NO_GET();
std::optional<uint16_t> ParseLXDT_FLIGHTS_NO_ANS(NMEAInputLine &line);

Message GenerateLXDT_FLIGHT_INFO_GET(uint16_t flight_index);
std::optional<FlightInfo> ParseLXDT_FLIGHT_INFO_ANS(NMEAInputLine &line);

Message GenerateLXDT_R_SWITCH_TOGGLE();
Message GenerateLXDT_R_DUAL_SET(bool is_dual_enabled);
Message GenerateLXDT_R_SPACING_SET(bool is_833);
}
}
#endif

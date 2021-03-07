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

#include "ProtocolDefinitions.hpp"
#include "ProtocolMatchers.hpp"
#include "Device/Driver/LXNavigation/Internals/LXNavigationData.hpp"

#include "util/StaticString.hxx"
#include "NMEA/InputLine.hpp"
#include "Engine/GlideSolvers/PolarCoefficients.hpp"

#include <vector>

class NMEAInputLine;
struct NMEAInfo;

namespace LXNavigation
{
namespace NMEAv1
{
extern const char* FLIGHT_DATA_PARAMETERS;
extern const char* BASIC_DEVICE_INFO;
extern const char* BASIC_GLIDE_INFO_PARAMETERS;
extern const char* ADVANCED_GLIDE_INFO_PARAMETERS;

using Message = NarrowString<60>;

constexpr int PFLX0_ONCE = -1;
constexpr int PFLX0_DISABLED = 0;
using PFLX0Request = std::vector<std::pair<NarrowString<6>, int> >;

template<Sentences sentence>
bool IsLineMatch(const NMEAInputLine& nmea_line)
{
  NMEAInputLine line(nmea_line);
  return MatchSentence<sentence>(line);
}

Message GeneratePFLX0(const PFLX0Request &request);

Message GeneratePFLX2ForMcReady(double mc);
Message GeneratePFLX2ForBugs(uint16_t bugs);
Message GeneratePFLX2ForVolume(unsigned volume);
Message GeneratePFLX2ForBallast(double fraction, double overload);
Message GeneratePFLX2ForPolar(const PolarCoefficients& polar);

void ParseLXWP0(NMEAInputLine &line, NMEAInfo &info);
DeviceInfo ParseLXWP1(NMEAInputLine &line);
std::tuple<GlideParameters, PolarCoefficients, int> ParseLXWP2(NMEAInputLine &line);
void ParseLXWP3(NMEAInputLine &line, NMEAInfo &info);

}
}
#endif


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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PROTOCOL_MATCHERS_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PROTOCOL_MATCHERS_HPP

#include "ProtocolDefinitions.hpp"

class NMEAInputLine;

namespace LXNavigation
{
template<Sentences sentence>
bool MatchSentence(NMEAInputLine& nmea_line);

template<NMEAv2::SentenceCode command>
bool MatchCommand(NMEAInputLine& nmea_line);

template<NMEAv2::SentenceAction status>
bool MatchStatus(NMEAInputLine& nmea_line);

//Sentence specializations
template<>
bool MatchSentence<Sentences::LXDT>(NMEAInputLine& nmea_line);

//Command specializations
template<>
bool MatchCommand<NMEAv2::SentenceCode::INFO>(NMEAInputLine& nmea_line);
template<>
bool MatchCommand<NMEAv2::SentenceCode::MC_BAL>(NMEAInputLine& nmea_line);
template<>
bool MatchCommand<NMEAv2::SentenceCode::OK>(NMEAInputLine& nmea_line);
template<>
bool MatchCommand<NMEAv2::SentenceCode::ERROR>(NMEAInputLine& nmea_line);

//Status specializations
template<>
bool MatchStatus<NMEAv2::SentenceAction::ANS>(NMEAInputLine& nmea_line);
}

#endif

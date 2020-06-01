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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PROTOCOL_DEFINITIONS_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PROTOCOL_DEFINITIONS_HPP

#include "Util/StaticString.hxx"

namespace LXNavigation
{
enum class Sentences
{
  LXWP0,
  LXWP1,
  LXWP2,
  LXWP3,
  LXDT
};
namespace NMEAv2
{
using Message = NarrowString<60>;

enum class SentenceCode
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

enum class SentenceAction
{
  GET,
  SET,
  ANS
};

}
}

#endif

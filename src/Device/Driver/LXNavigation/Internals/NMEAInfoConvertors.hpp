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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_NMEAINFO_CONVERTERS_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_NMEAINFO_CONVERTERS_HPP

#include "Device/Driver/LXNavigation/Internals/LXNavigationData.hpp"
#include "NMEA/InputLine.hpp"

struct NMEAInfo;
struct PolarCoefficients;

namespace LXNavigation
{
template<typename T>
void ReadElement(NMEAInputLine& line, std::optional<T>& out_value)
{
  T value;
  out_value = std::optional<T>{};
  if(line.ReadChecked(value))
    out_value = value;
}

void ConvertToNMEAInfo(GlideParameters &parameters, NMEAInfo &info);
void ConvertToNMEAInfo(DeviceParameters &parameters, NMEAInfo &info);
void ConvertToNMEAInfo(PolarCoefficients &parameters, NMEAInfo &info);



}

#endif

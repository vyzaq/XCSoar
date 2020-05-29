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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PFLX0_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_PFLX0_HPP

#include "Util/StaticString.hxx"
#include <vector>
#include <utility>

namespace LXNavigation
{
constexpr int PFLX0_ONCE = -1;
constexpr int PFLX0_DISABLED = 0;
using PFLX0Message = NarrowString<60>;
using PFLX0Request = std::vector<std::pair<NarrowString<6>, int> >;
PFLX0Message GeneratePFLX0(const PFLX0Request &request);
}

#endif

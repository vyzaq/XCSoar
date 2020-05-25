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

#include "LXNavigationDevice.hpp"
#include "Device/Port/Port.hpp"
#include "Operation/Operation.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"

#include <OS/Path.hpp>

void
LXNavigationDevice::LinkTimeout()
{
  busy = false;
  std::lock_guard<Mutex> lock(mutex);
  device_baud_rate = 0;
}

bool
LXNavigationDevice::EnableNMEA(OperationEnvironment &env)
{
  unsigned cached_device_baud_rate;

  {
    std::lock_guard<Mutex> lock(mutex);

    cached_device_baud_rate = device_baud_rate;
    device_baud_rate = 0;
    busy = false;
  }

  if (cached_device_baud_rate != 0)
    port.SetBaudrate(cached_device_baud_rate);

  port.Flush();

  return true;
}

bool
LXNavigationDevice::ParseNMEA(const char *String, NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(String))
    return false;
  return false;
}

bool
LXNavigationDevice::PutBallast(double fraction, double overload, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutBugs(double bugs, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutMacCready(double mc, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutQNH(const AtmosphericPressure &pres, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutVolume(unsigned volume, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutActiveFrequency(RadioFrequency frequency, const TCHAR *name, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutStandbyFrequency(RadioFrequency frequency, const TCHAR *name, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::Declare(const Declaration &declaration, const Waypoint *home, OperationEnvironment &env)
{
  return false;
}

void
LXNavigationDevice::OnSysTicker()
{
}

bool
LXNavigationDevice::ReadFlightList(RecordedFlightList &flight_list, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::DownloadFlight(const RecordedFlightInfo &flight, Path path, OperationEnvironment &env)
{
  return false;
}

LXNavigationDevice::LXNavigationDevice(Port &_port, unsigned baud_rate, unsigned bulk_baud_rate)
    : port(_port), device_bulk_baud_rate(bulk_baud_rate), device_baud_rate(baud_rate),
      busy(false) {}

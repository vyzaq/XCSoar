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

#include "NMEAv2Protocol.hpp"
#include "NMEAv1Protocol.hpp"

#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Operation/Operation.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"
#include <OS/Path.hpp>

#include <utility>

namespace LXNavigation
{
namespace
{
bool IsDeviceSupported(const DeviceInfo& device)
{
  if(device.name == "LX Eos 57")
  {
    return !(device.sw_version < 1.9);
  }
  else if(device.name == "LX Eos 80")
  {
    return !(device.sw_version < 1.5);
  }
  else if(device.name == "LX Era")
  {
    return !(device.sw_version < 1.5);
  }
  else if(device.name == "LX Colibri X")
  {
    return !(device.sw_version < 1.5);
  }
  else if(device.name == "LX Zeus")
  {
    return !(device.sw_version < 5);
  }
  return false;
}
}

LXNavigationDevice::LXNavigationDevice(Port &communication_port, unsigned baud_rate, unsigned bulk_baud_rate)
    : port(communication_port)
    , device_bulk_baud_rate(bulk_baud_rate)
    , device_baud_rate(baud_rate)
    , state(State::UNKNOWN) {}

void
LXNavigationDevice::LinkTimeout()
{
}

bool
LXNavigationDevice::EnableNMEA(OperationEnvironment &env)
{
  state = State::DEVICE_INIT;
  port.SetBaudrate(device_baud_rate);
  port.Flush();
  NMEAv1::PFLX0Request request;
  request.emplace_back(NMEAv1::FLIGHT_DATA_PARAMETERS, 1);
  request.emplace_back(NMEAv1::BASIC_GLIDE_INFO_PARAMETERS, 10);
  request.emplace_back(NMEAv1::BASIC_DEVICE_INFO, 60);
  request.emplace_back(NMEAv1::ADVANCED_GLIDE_INFO_PARAMETERS, NMEAv1::PFLX0_DISABLED);
  {
    const std::lock_guard<Mutex> lock(mutex);
    PortWriteNMEA(port, NMEAv1::GeneratePFLX0(request), env); //quering device info to get logged - debug only
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_INFO_GET(), env); //quering device info to get logged - debug only
    state = State::PROCESSING_NMEA;
  }
  return true;
}

bool
LXNavigationDevice::ParseNMEA(const char *string, NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(string))
    return false;

  NMEAInputLine nmea_line(string);
  switch (state) {
  case State::PROCESSING_NMEA:
  {
    if(NMEAv1::IsLineMatch(nmea_line, Sentences::LXWP1) ||
       NMEAv2::IsLineMatch(nmea_line, Sentences::LXDT, NMEAv2::SentenceAction::INFO, NMEAv2::SentenceCode::ANS))
    {
      DeviceInfo device_info{};
      if(NMEAv1::IsLineMatch(nmea_line, Sentences::LXWP1))
        device_info = NMEAv1::ParseLXWP1(nmea_line);
      else
        device_info = NMEAv2::ParseLXDT_INFO_ANS(nmea_line);
      if(!IsDeviceSupported(device_info)) {
        state = State::NOT_SUPPORTED;
        return true;
      }
    }
    else if(NMEAv1::IsLineMatch(nmea_line, Sentences::LXWP0))
    {
      NMEAv1::ParseLXWP0(nmea_line, info);
      return true;
    }
    else if(NMEAv1::IsLineMatch(nmea_line, Sentences::LXWP2))
    {
      NMEAv1::ParseLXWP2(nmea_line, info);
      return true;
    }
    else if(NMEAv1::IsLineMatch(nmea_line, Sentences::LXWP3))
    {
      NMEAv1::ParseLXWP3(nmea_line, info);
      return true;
    }
    return true;
  }
  default:
    break;
  }
  return false;
}

bool
LXNavigationDevice::PutBallast(double fraction, double overload, OperationEnvironment &env)
{
  PortWriteNMEA(port, NMEAv1::GeneratePFLX2ForBallast(fraction, overload), env);
  return true;
}

bool
LXNavigationDevice::PutBugs(double bugs, OperationEnvironment &env)
{
  PortWriteNMEA(port, NMEAv1::GeneratePFLX2ForBugs(bugs), env);
  return true;
}

bool
LXNavigationDevice::PutMacCready(double mc, OperationEnvironment &env)
{
  PortWriteNMEA(port, NMEAv1::GeneratePFLX2ForMcReady(mc), env);
  return true;
}

bool
LXNavigationDevice::PutQNH(const AtmosphericPressure &pres, OperationEnvironment &env)
{
  return false;
}

bool
LXNavigationDevice::PutVolume(unsigned volume, OperationEnvironment &env)
{
  PortWriteNMEA(port, NMEAv1::GeneratePFLX2ForVolume(volume), env);
  return true;
}

bool
LXNavigationDevice::PutActiveFrequency(RadioFrequency frequency, const TCHAR *name, OperationEnvironment &env)
{
  RadioParameters parameters{};
  parameters.active_freq = static_cast<float>(frequency.GetKiloHertz()) / 1000;
  PortWriteNMEA(port, NMEAv2::GenerateLXDT_RADIO_SET(parameters), env);
  return true;
}

bool
LXNavigationDevice::PutStandbyFrequency(RadioFrequency frequency, const TCHAR *name, OperationEnvironment &env)
{
  RadioParameters parameters{};
  parameters.standby_freq = static_cast<float>(frequency.GetKiloHertz()) / 1000;
  PortWriteNMEA(port, NMEAv2::GenerateLXDT_RADIO_SET(parameters), env);
  return true;
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
}

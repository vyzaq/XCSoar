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
#include "NMEAInfoConvertors.hpp"

#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Device/Declaration.hpp"
#include "Device/RecordedFlight.hpp"
#include "Operation/Operation.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"
#include <system/Path.hpp>
#include "Device/Util/NMEAReader.hpp"
#include "time/TimeoutClock.hpp"

#include <utility>
#include <chrono>

namespace LXNavigation
{
namespace
{
bool IsDeviceSupported(const DeviceInfo &device)
{
  if(device.name == "LX Eos 57")
  {
    return !(device.sw_version < 1.9);
  }
  else if(device.name == "LX Era" ||
          device.name == "LX Eos 80" ||
          device.name == "LX Eos")
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

TurnpointZone ConvertToZone(const Declaration::TurnPoint &turnpoint)
{
  TurnpointZone zone{};
  zone.direction = Direction::Symmetric;
  zone.is_auto_next = true;
  zone.r1 = turnpoint.radius;
  switch (turnpoint.shape) {
  case Declaration::TurnPoint::CYLINDER:
    zone.a1 = 180;
    break;
  case Declaration::TurnPoint::DAEC_KEYHOLE:
  case Declaration::TurnPoint::SECTOR:
    zone.a1 = 45;
    break;
  case Declaration::TurnPoint::LINE:
    zone.direction = Direction::ToPrevious;
    zone.is_line = true;
    zone.r1 = turnpoint.radius*2;
    break;
  }
  return zone;
}

int GetNumberOfFlights(Port &port, PortNMEAReader &reader,
                       OperationEnvironment &env, TimeoutClock timeout)
{
  reader.Flush();
  if(!PortWriteNMEA(port, NMEAv2::GenerateLXDT_FLIGHTS_NO_GET(), env))
    return -1;
  port.Drain();
  auto result = reader.ExpectLine(NMEAv2::LXDT_ANS_FLIGHTS_NO, timeout);
  if(!result)
    return -1;

  NMEAInputLine line(result);
  line.Skip();
  auto number_of_flights = NMEAv2::ParseLXDT_FLIGHTS_NO_ANS(line);
  return number_of_flights ? *number_of_flights : -1;
}

bool WaitAndReadAnswer(PortNMEAReader& reader, TimeoutClock timeout)
{
  reader.Flush();
  auto result = reader.ExpectLine(NMEAv2::LXDT_ANS, timeout);
  if(!result)
    return false;
  NMEAInputLine line(result);
  auto answer = NMEAv2::ParseLXDT_ANS_Status(line);
  return answer.first;
}

std::optional<FlightInfo> WaitAndReadFlightInfo(PortNMEAReader& reader, TimeoutClock timeout)
{
  reader.Flush();
  auto result = reader.ExpectLine(NMEAv2::LXDT_ANS_FLIGHT_INFO, timeout);
  if(!result)
    return {};
  NMEAInputLine line(result);
  line.Skip();
  return NMEAv2::ParseLXDT_FLIGHT_INFO_ANS(line);
}

void DeviceInfoIntoNMEA(const DeviceInfo& device_info, NMEAInfo& info)
{
  info.device.product = device_info.name;
  info.device.serial.Format("%d", device_info.serial);
  info.device.software_version.Format("%.2f", device_info.sw_version);
  info.device.hardware_version.Format("%.2f", device_info.hw_version);
}

}

LXNavigationDevice::LXNavigationDevice(Port &communication_port)
    : port(communication_port)
    , state(State::UNKNOWN) {}

void
LXNavigationDevice::LinkTimeout()
{
}

bool
LXNavigationDevice::EnableNMEA(OperationEnvironment &env)
{
  state = State::DEVICE_INIT;
  port.Flush();
  NMEAv1::PFLX0Request request;
  request.emplace_back(NMEAv1::FLIGHT_DATA_PARAMETERS, 1);
  request.emplace_back(NMEAv1::BASIC_GLIDE_INFO_PARAMETERS, 10);
  request.emplace_back(NMEAv1::BASIC_DEVICE_INFO, NMEAv1::PFLX0_ONCE);
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
    if(NMEAv1::IsLineMatch<Sentences::LXWP1>(nmea_line))
    {
      nmea_line.Skip();
      auto device_info = NMEAv1::ParseLXWP1(nmea_line);
      DeviceInfoIntoNMEA(device_info, info);
      if(!IsDeviceSupported(device_info)) {
        state = State::NOT_SUPPORTED;
      }
      return true;
    }
    else if(NMEAv2::IsLineMatch<Sentences::LXDT, NMEAv2::SentenceCode::INFO, NMEAv2::SentenceAction::ANS>(nmea_line))
    {
      nmea_line.Skip(3);
      auto device_info = NMEAv2::ParseLXDT_INFO_ANS(nmea_line);
      DeviceInfoIntoNMEA(device_info, info);
      if(!IsDeviceSupported(device_info)) {
        state = State::NOT_SUPPORTED;
      }
      return true;
    }
    else if(NMEAv1::IsLineMatch<Sentences::LXWP0>(nmea_line))
    {
      nmea_line.Skip();
      NMEAv1::ParseLXWP0(nmea_line, info);
      return true;
    }
    else if(NMEAv1::IsLineMatch<Sentences::LXWP2>(nmea_line))
    {
      nmea_line.Skip();
      auto parameters = NMEAv1::ParseLXWP2(nmea_line);
      ConvertToNMEAInfo(std::get<0>(parameters), info);
      ConvertToNMEAInfo(std::get<1>(parameters), info);
      auto volume = std::get<2>(parameters);
      if(volume >= 0)
        info.settings.ProvideVolume(volume, info.clock);
      return true;
    }
    else if(NMEAv1::IsLineMatch<Sentences::LXWP3>(nmea_line))
    {
      nmea_line.Skip();
      NMEAv1::ParseLXWP3(nmea_line, info);
      return true;
    }
    else if(NMEAv2::IsLineMatch<Sentences::LXDT, NMEAv2::SentenceCode::MC_BAL, NMEAv2::SentenceAction::ANS>(nmea_line))
    {
      nmea_line.Skip(3);
      auto parameters = NMEAv2::ParseLXDT_MC_BAL_ANS(nmea_line);
      ConvertToNMEAInfo(parameters.first, info);
      ConvertToNMEAInfo(parameters.second, info);
      return true;
    }
    else if(NMEAv2::IsLineMatch<Sentences::LXDT, NMEAv2::SentenceCode::OK, NMEAv2::SentenceAction::ANS>(nmea_line))
    {
      return true;
    }
    else if(NMEAv2::IsLineMatch<Sentences::LXDT, NMEAv2::SentenceCode::ERROR, NMEAv2::SentenceAction::ANS>(nmea_line))
    {
      return true;
    }
    break;
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
  PortWriteNMEA(port, NMEAv1::GeneratePFLX2ForBugs(100 - static_cast<unsigned>(bugs*100)), env);
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
  if(declaration.turnpoints.size() > TurnpointData::max_points)
    return false;
  env.SetProgressRange(declaration.turnpoints.size() + 2 + home ? 1 : 0);
  unsigned current_progress = 0;
  env.SetProgressPosition(current_progress);
  port.StopRxThread();

  {
    PilotInfo pilot_info;
    pilot_info.name = declaration.pilot_name; //split in the space?
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_PILOT_SET(pilot_info), env);
    env.SetProgressPosition(++current_progress);
    port.Drain();
    if(!port.ExpectString(NMEAv2::LXDT_ANS_OK, env))
      return false;
  }
  {
    GliderInfo glider_info;
    glider_info.reg_no = declaration.aircraft_registration;
    glider_info.comp_id = declaration.competition_id;
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_GLIDER_SET(glider_info), env);
    env.SetProgressPosition(++current_progress);
    port.Drain();
    if(!port.ExpectString(NMEAv2::LXDT_ANS_OK, env))
      return false;
  }

  TurnpointData data;
  auto turnpoints_quantity = declaration.turnpoints.size();
  data.total_tp_count = turnpoints_quantity + 2; // +2 because according to spec number have to append takeoff and landing
  if(home) {
    data.id = 0;
    data.name.SetASCII(home->name.c_str());
    data.location = home->location;
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_TP_SET(data), env);
    env.SetProgressPosition(++current_progress);
    port.Drain();
    if(!port.ExpectString(NMEAv2::LXDT_ANS_OK, env))
      return false;
  }

  for(unsigned i = 0; i < turnpoints_quantity; ++i)
  {
    const auto &turnpoint = declaration.turnpoints.at(i);
    data.id = i+1;
    data.name.SetASCII(turnpoint.waypoint.name.c_str());
    data.location = turnpoint.waypoint.location;

    PortWriteNMEA(port, NMEAv2::GenerateLXDT_TP_SET(data), env);
    if(!port.ExpectString(NMEAv2::LXDT_ANS_OK, env))
      return false;
    auto zone_data = ConvertToZone(turnpoint);
    zone_data.id = data.id;
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_ZONE_SET(zone_data), env);
    port.Drain();
    if(!port.ExpectString(NMEAv2::LXDT_ANS_OK, env))
      return false;
    env.SetProgressPosition(++current_progress);
  }
  return true;
}

void
LXNavigationDevice::OnSysTicker()
{
}

bool
LXNavigationDevice::ReadFlightList(RecordedFlightList &flight_list, OperationEnvironment &env)
{
  port.StopRxThread();

  NMEAv1::PFLX0Request request;
  request.emplace_back(NMEAv1::FLIGHT_DATA_PARAMETERS, NMEAv1::PFLX0_DISABLED);
  request.emplace_back(NMEAv1::BASIC_GLIDE_INFO_PARAMETERS, NMEAv1::PFLX0_DISABLED);
  request.emplace_back(NMEAv1::BASIC_DEVICE_INFO, NMEAv1::PFLX0_DISABLED);
  request.emplace_back(NMEAv1::ADVANCED_GLIDE_INFO_PARAMETERS, NMEAv1::PFLX0_DISABLED);
  PortWriteNMEA(port, NMEAv1::GeneratePFLX0(request), env); //quering device info to get logged - debug only
  PortWriteNMEA(port, "LXDT,SET,BC_INT,ALL,0", env); //quering device info to get logged - debug only
  port.Drain();
  PortNMEAReader reader(port, env);
  {
    TimeoutClock timeout(std::chrono::seconds(5));
    if(!WaitAndReadAnswer(reader, timeout))
      return false;
  }

  int flights_quantity = 0;
  {
    TimeoutClock timeout(std::chrono::seconds(5));
    flights_quantity = GetNumberOfFlights(port, reader, env, timeout);
    if (flights_quantity < 0)
      return false;
  }

  env.SetProgressRange(flights_quantity);

  for(int i = 0; i < flights_quantity; ++i)
  {
    {
      TimeoutClock timeout(std::chrono::milliseconds(50));
      reader.ExpectLine(NMEAv2::LXDT_ANS_FLIGHT_INFO, timeout);
    }

    env.SetProgressPosition(i);
    PortWriteNMEA(port, NMEAv2::GenerateLXDT_FLIGHT_INFO_GET(i+1), env);
    port.Drain();
    TimeoutClock timeout(std::chrono::seconds(5));
    auto flight_info = WaitAndReadFlightInfo(reader, timeout);
    if(!flight_info)
      return false;
    RecordedFlightInfo app_flight_info;
    app_flight_info.date = flight_info->date;
    app_flight_info.start_time = flight_info->take_off;
    app_flight_info.end_time = flight_info->landing;
    app_flight_info.internal.lx_navigation = flight_info->flight_id;
    flight_list.emplace_back(app_flight_info);
  }
  return true;
}

bool
LXNavigationDevice::DownloadFlight(const RecordedFlightInfo &flight, Path path, OperationEnvironment &env)
{
  if (flight.internal.lx_navigation <= 0)
    return false;

  state = State::DOWNLOADING_FLIGHT;
  port.StopRxThread();
  port.Drain();
  return false;
}

void LXNavigationDevice::OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated)
{
  NullOperationEnvironment env;

//  PutIdealPolar(calculated, env);
//  PutRealPolar(calculated, env);

//  if (!_mc_valid) {
//    // this is the MacCready at start up
//    _mc = calculated.glide_polar_safety.GetMC();
//    _mc_valid = true;
//    RepeatMacCready(env);
//    }
}

}

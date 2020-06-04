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

#ifndef XCSOAR_DEVICE_DRIVER_LXNAVIGATION_DATA_HPP
#define XCSOAR_DEVICE_DRIVER_LXNAVIGATION_DATA_HPP

#include "Util/StaticString.hxx"
#include "Geo/GeoPoint.hpp"
#include "Time/BrokenDate.hpp"
#include "Time/BrokenTime.hpp"

#include <optional>
#include <chrono>

namespace LXNavigation
{

struct BasicFlightInfo
{
  bool is_logger_running;
  float tas;
  float altitude;
  float vario6;
  uint16_t heading;
  float wind_direction;
  float wind_speed;
};

struct DeviceInfo
{
  NarrowString<60> name;
  uint32_t serial;
  double sw_version;
  double hw_version;
};

struct GlideParameters
{
  std::optional<double> mac_cready;
  std::optional<double> load_factor;
  std::optional<int> bugs;
};

enum class SpeedCommandMode
{
  Manual,
  Circling,
  Speed
};

enum class SpeedCommandSwitchMode
{
  Off,
  On,
  Toggle
};

struct SpeedCommandParameters
{
  int16_t alt_offset;
  SpeedCommandMode sc_mode;
  float filter;
  uint16_t te_level;
  uint16_t int_time;
  uint8_t range;
  float silence;
  SpeedCommandSwitchMode switch_mode;
  uint16_t speed;
  NarrowString<60> polar_name;
};

enum class TurnpointType
{
  Point = 1,
  Landing,
  Takeoff
};

struct TurnpointData
{
  static constexpr uint8_t max_points = 12;

  uint16_t id;
  uint8_t total_tp_count; //set only
  TurnpointType type; //get only
  GeoPoint location;
  NarrowString<30> name;
};

enum class Direction
{
  Symmetric,
  Fixed,
  ToNext,
  ToPrevious,
  ToStart
};

struct TurnpointZone
{
  uint16_t id;
  Direction direction;
  bool is_auto_next;
  bool is_line;
  uint16_t a1;
  uint16_t a2;
  uint16_t a21;
  uint16_t r1;
  uint16_t r2;
  uint16_t elevation;
};

enum class GliderClass
{
  None,
  Standard,
  Meter15,
  Open,
  Meter18,
  World,
  Double,
  MotorGL
};

struct GliderInfo
{
  NarrowString<13> polar_name;
  NarrowString<32> reg_no;
  NarrowString<5> comp_id;
  GliderClass glider_class;
};

struct PilotInfo
{
  NarrowString<64> name;
  NarrowString<64> surname;
};

struct TaskParameters
{
  bool finish_1000;
  uint16_t finish_alt_offset;
  NarrowString<6> aat_time;
};

struct DeviceParameters
{
  std::optional<int> brightness;
  std::optional<int> vario_vol;
  std::optional<int> sc_vol;
};

struct RadioParameters
{
  double active_freq;
  double standby_freq;
  int volume;
  int squelch;
  int vox;
};

struct FlightInfo
{
  uint16_t flight_id;
  NarrowString<10> filename;
  BrokenDate date;
  BrokenTime take_off;
  BrokenTime landing;
  NarrowString<13> pilot_name;
  NarrowString<13> pilot_surname;
  NarrowString<9> reg_no;
  NarrowString<9> comp_id;
  float min_gforce;
  float max_gforce;
  uint16_t max_alt;
  double max_ias;
};

}

#endif

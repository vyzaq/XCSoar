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

namespace LXNavigation
{

enum class Status
{
  Ok,
  Error
};
struct StatusResult
{
  Status status;
  NarrowString<60> description;
};

struct DeviceInfo
{
  NarrowString<60> name;
  u_int32_t serial;
  float sw_version;
  float hw_version;
};

struct GlideParameters
{
  float mc_ready;
  float load_factor;
  u_int16_t bugs;
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
  u_int16_t te_level;
  u_int16_t int_time;
  u_int8_t range;
  float silence;
  SpeedCommandSwitchMode switch_mode;
  u_int16_t speed;
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
  static constexpr u_int8_t max_points = 12;

  u_int16_t id;
  u_int8_t total_tp_count;
  TurnpointType type;
  GeoPoint location;
  NarrowString<10> name;
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
  u_int16_t id;
  Direction direction;
  bool is_auto_next;
  bool is_line;
  u_int16_t a1;
  u_int16_t a2;
  u_int16_t a21;
  u_int16_t r1;
  u_int16_t r2;
  u_int16_t elevation;
};

enum class GliderClass
{
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
  NarrowString<9> reg_no;
  NarrowString<5> comp_id;
  GliderClass glider_class;
};

struct PilotInfo
{
  NarrowString<20> name;
  NarrowString<20> surname;
};

struct TaskParameters
{
  bool finish_1000;
  u_int16_t finish_alt_offset;
  double aat_time_sec;
};

struct DeviceParameters
{
  u_int8_t brightness;
  u_int8_t vario_vol;
  u_int8_t sc_vol;
};

struct RadioParameters
{
  float active_freq;
  float standby_freq;
  u_int16_t volume;
  u_int16_t squelch;
  u_int16_t vox;
};

struct FlightInfo
{
u_int16_t flight_id;
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
u_int16_t max_alt;
double max_ias;
};

}

#endif

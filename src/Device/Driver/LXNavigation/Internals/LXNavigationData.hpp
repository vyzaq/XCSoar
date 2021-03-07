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

#include "util/StaticString.hxx"
#include "Geo/GeoPoint.hpp"
#include "time/BrokenDate.hpp"
#include "time/BrokenTime.hpp"

#include <optional>
#include <chrono>

namespace LXNavigation
{

struct BasicFlightInfo
{
  bool is_logger_running = false;
  float tas = 0;
  float altitude = 0;
  float vario6 = 0;
  uint16_t heading = 0;
  float wind_direction = 0;
  float wind_speed = 0;
};

struct DeviceInfo
{
  NarrowString<60> name = {};
  uint32_t serial = 0;
  double sw_version = 0;
  double hw_version = 0;
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
  int16_t alt_offset = 0;
  SpeedCommandMode sc_mode = SpeedCommandMode::Manual;
  float filter = 0;
  uint16_t te_level = 0;
  uint16_t int_time = 0;
  uint8_t range = 0;
  float silence = 0;
  SpeedCommandSwitchMode switch_mode = SpeedCommandSwitchMode::Toggle;
  uint16_t speed = 0;
  NarrowString<60> polar_name = {};
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

  uint16_t id = 0;
  uint8_t total_tp_count = 0; //set only
  TurnpointType type = TurnpointType::Point; //get only
  GeoPoint location = {};
  NarrowString<30> name = {};
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
  uint16_t id = 0;
  Direction direction = Direction::Symmetric;
  bool is_auto_next = false;
  bool is_line = false;
  uint16_t a1 = 0;
  uint16_t a2 = 0;
  uint16_t a21 = 0;
  uint16_t r1 = 0;
  uint16_t r2 = 0;
  uint16_t elevation = 0;
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
  NarrowString<13> polar_name = {};
  NarrowString<32> reg_no = {};
  NarrowString<5> comp_id = {};
  GliderClass glider_class = GliderClass::None;
};

struct PilotInfo
{
  NarrowString<64> name = {};
  NarrowString<64> surname = {};
};

struct TaskParameters
{
  bool finish_1000 = false;
  uint16_t finish_alt_offset = 0;
  NarrowString<6> aat_time = {};
};

struct DeviceParameters
{
  std::optional<int> brightness;
  std::optional<int> vario_vol;
  std::optional<int> sc_vol;
};

struct RadioParameters
{
  double active_freq = 0;
  double standby_freq = 0;
  int volume = 0;
  int squelch = 0;
  int vox = 0;
};

struct FlightInfo
{
  uint16_t flight_id = 0;
  NarrowString<10> filename = {};
  BrokenDate date = {};
  BrokenTime take_off = {};
  BrokenTime landing = {};
  NarrowString<13> pilot_name = {};
  NarrowString<13> pilot_surname = {};
  NarrowString<9> reg_no = {};
  NarrowString<9> comp_id = {};
  float min_gforce = 0;
  float max_gforce = 0;
  uint16_t max_alt = 0;
  double max_ias = 0;
};

}

#endif

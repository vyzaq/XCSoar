/* Copyright_License {

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

#include "Device/Driver/LXNavigation/Internals/NMEAv1Protocol.hpp"
#include "Device/Driver/LXNavigation/Internals/NMEAv2Protocol.hpp"
#include "Device/Driver/LXNavigation/Internals/BinaryProtocol.hpp"
#include "TestUtil.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"
#include "Units/System.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void
TestPFLX0()
{
  { //for requests
    LXNavigation::NMEAv1::PFLX0Request expected_messages;
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP0", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP1", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP2", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP3", 1));
    ok1(LXNavigation::NMEAv1::GeneratePFLX0(expected_messages) ==
          "PFLX0,LXWP0,1,LXWP1,1,LXWP2,1,LXWP3,1");
  }
  { //empty not allowed
    LXNavigation::NMEAv1::PFLX0Request expected_messages;
    ok1(LXNavigation::NMEAv1::GeneratePFLX0(expected_messages) == "");
  }

  { //more than 4 not allowed
    LXNavigation::NMEAv1::PFLX0Request expected_messages;
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP0", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP1", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP2", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP3", 1));
    expected_messages.emplace_back(std::pair<NarrowString<6>, int>("LXWP3", 1));
    ok1(LXNavigation::NMEAv1::GeneratePFLX0(expected_messages) == "");
  }
}

static void
TestPFLX2()
{
  ok1(LXNavigation::NMEAv1::GeneratePFLX2ForMcReady(2.4) ==
        "PFLX2,2.4,,,,,,");
  ok1(LXNavigation::NMEAv1::GeneratePFLX2ForBugs(34) ==
        "PFLX2,,,34,,,,");
  ok1(LXNavigation::NMEAv1::GeneratePFLX2ForVolume(25) ==
        "PFLX2,,,,,,,25");
  ok1(LXNavigation::NMEAv1::GeneratePFLX2ForBallast(22, 1.12) ==
        "PFLX2,,1.12,,,,,");
}

static void
TestLXWP0()
{
  {
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line("Y,119.4,1717.6,0.02,0.02,0.02,0.02,0.02,0.02,,022,107.2");
    LXNavigation::NMEAv1::ParseLXWP0(line, info);
    ok1(info.airspeed_available);
    ok1(equals(info.true_airspeed, Units::ToSysUnit(119.4, Unit::KILOMETER_PER_HOUR)));
    ok1(info.baro_altitude_available);
    ok1(equals(info.baro_altitude, 1717.6));
    ok1(info.total_energy_vario_available);
    ok1(equals(info.total_energy_vario, 0.02));
    ok1(info.external_wind_available);
    ok1(info.external_wind.bearing == Angle::Degrees(22));
    ok1(equals(info.external_wind.norm, Units::ToSysUnit(107.2, Unit::KILOMETER_PER_HOUR)));
    ok1(!info.heading_available);
  }
  {
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line("Y,,,,,,,,,212,,");
    LXNavigation::NMEAv1::ParseLXWP0(line, info);
    ok1(!info.airspeed_available);
    ok1(!info.baro_altitude_available);
    ok1(!info.total_energy_vario_available);
    ok1(info.external_wind_available);
    ok1(info.external_wind.bearing == Angle::Degrees(0));
    ok1(equals(info.external_wind.norm, 0));
    ok1(info.heading_available);
    ok1(info.heading == Angle::Degrees(212));
  }
}

static void
TestLXWP1()
{
  NMEAInputLine line("LX Eos,34949,1.5,1.4");
  auto device_info = LXNavigation::NMEAv1::ParseLXWP1(line);
  ok1(device_info.name == "LX Eos");
  ok1(device_info.serial == 34949);
  ok1(equals(device_info.sw_version, 1.5));
  ok1(equals(device_info.hw_version, 1.4));
}

static void
TestLXWP2()
{
  {
    NMEAInputLine line("1.5,1.11,13,2.96,-3.03,1.35,45");

    LXNavigation::GlideParameters glide_params;
    PolarCoefficients polar;
    int volume = -1;
    std::tie(glide_params, polar, volume) = LXNavigation::NMEAv1::ParseLXWP2(line);
    ok1(glide_params.mac_cready);
    ok1(equals(*glide_params.mac_cready, 1.5));
    ok1(glide_params.load_factor);
    ok1(equals(*glide_params.load_factor, 1.11));
    ok1(glide_params.bugs);
    ok1(*glide_params.bugs == 13);
    ok1(volume == 45);
  }
  {
    NMEAInputLine line(",,,,,,");
    LXNavigation::GlideParameters glide_params;
    PolarCoefficients polar;
    int volume = -1;
    std::tie(glide_params, polar, volume) = LXNavigation::NMEAv1::ParseLXWP2(line);
    ok1(!glide_params.mac_cready);
    ok1(!glide_params.load_factor);
    ok1(!glide_params.bugs);
    ok1(volume == -1);
  }
}

static void
TestLXWP3()
{
  {
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line("200,2,5.0,0,29,20,10.0,1.3,1,120,0,KA6e,0");
    LXNavigation::NMEAv1::ParseLXWP3(line, info);
    ok1(info.settings.qnh_available);
    ok1(equals(info.settings.qnh.GetHectoPascal(), 1020.59));
  }
  {
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line(",2,5.0,0,29,20,10.0,1.3,1,120,0,KA6e,0");
    LXNavigation::NMEAv1::ParseLXWP3(line, info);
    ok1(!info.settings.qnh_available);
  }
}

static void
TestLXBC()
{
}

static void
TestLXDT_ANS_Status()
{
  {
    NMEAInputLine line("ERROR,Parameter count mismatch");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ANS_Status(line);
    ok1(!parsing_result.first);
    ok1(parsing_result.second == "Parameter count mismatch");
  }
  {
    NMEAInputLine line("OK");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ANS_Status(line);
    ok1(parsing_result.first);
    ok1(parsing_result.second == "");
  }
}

static void
TestLXDT_INFO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_INFO_GET() == "LXDT,GET,INFO");
  {
    NMEAInputLine line("LX Era,34949,1.4,1.1,0-[0],00,Empty,Empty");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_INFO_ANS(line);
    ok1(parsing_result.name == "LX Era");
    ok1(parsing_result.serial == 34949);
    ok1(equals(parsing_result.sw_version, 1.4));
    ok1(equals(parsing_result.hw_version, 1.1));
  }
}

static void
TestLXDT_TP()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_TP_GET(2) == "LXDT,GET,TP,2");
  {
    LXNavigation::TurnpointData turnpoint;
    turnpoint.id = 0;
    turnpoint.name = "NOVO MESTO";
    turnpoint.location = GeoPoint(Angle::Degrees(15.1127), Angle::Degrees(45.810283));
    turnpoint.total_tp_count = 5;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_TP_SET(turnpoint) == "LXDT,SET,TP,0,5,2748616,906762,NOVO MESTO");
  }
  {
    NMEAInputLine line("2,2,2748616,906762,NOVO MESTO ");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_TP_ANS(line);
    ok1(parsing_result);
    ok1(parsing_result->id == 2);
    ok1(parsing_result->name == "NOVO MESTO ");
    ok1(equals(parsing_result->location.longitude.Degrees(), 15.1127));
    ok1(equals(parsing_result->location.latitude.Degrees(), 45.810283));
    ok1(parsing_result->type == LXNavigation::TurnpointType::Landing);
  }
  {
    NMEAInputLine line(",,2748616,906762,NOVO MESTO ");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_TP_ANS(line);
    ok1(!parsing_result);
  }
}

static void
TestLXDT_ZONE()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_ZONE_GET(2) == "LXDT,GET,ZONE,2");
  {
    LXNavigation::TurnpointZone zone;
    zone.id = 2;
    zone.direction = LXNavigation::Direction::Fixed;
    zone.is_auto_next = true;
    zone.is_line = true;
    zone.a1 = 90;
    zone.a2 = 60;
    zone.a21 = 309;
    zone.r1 = 5000;
    zone.r2 = 3500;
    zone.elevation = 174;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_ZONE_SET(zone) == "LXDT,SET,ZONE,2,1,1,1,90,60,309,5000,3500,174");
  }
  {
    NMEAInputLine line("2,3,0,1,90,60,309,5000,3500,174");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ZONE_ANS(line);
    ok1(parsing_result);
    ok1(parsing_result->id == 2);
    ok1(parsing_result->direction == LXNavigation::Direction::ToPrevious);
    ok1(parsing_result->is_auto_next == false);
    ok1(parsing_result->is_line);
    ok1(parsing_result->a1 == 90);
    ok1(parsing_result->a2 == 60);
    ok1(parsing_result->a21 == 309);
    ok1(parsing_result->r1 == 5000);
    ok1(parsing_result->r2 == 3500);
    ok1(parsing_result->elevation == 174);
  }
  {
    NMEAInputLine line("2,,0,,90,60,309,5000,3500,174");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ZONE_ANS(line);
    ok1(!parsing_result);
  }
}

static void
TestLXDT_GLIDER()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_GLIDER_GET() == "LXDT,GET,GLIDER");

  {
    LXNavigation::GliderInfo glider;
    glider.reg_no = "D-LKXD";
    glider.comp_id = "XD";
    glider.glider_class = LXNavigation::GliderClass::Open;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_GLIDER_SET(glider) == "LXDT,SET,GLIDER,D-LKXD,XD,OPEN");
  }
  {
    NMEAInputLine line("JS3 15m,D-KLXD,XD,OPEN");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_GLIDER_ANS(line);
    ok1(parsing_result.reg_no == "D-KLXD");
    ok1(parsing_result.comp_id == "XD");
    ok1(parsing_result.polar_name == "JS3 15m");
    ok1(parsing_result.glider_class == LXNavigation::GliderClass::Open);
  }
}

static void
TestLXDT_PILOT()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_PILOT_GET() == "LXDT,GET,PILOT");
  {
    LXNavigation::PilotInfo pilot;
    pilot.name = "ACE";
    pilot.surname = "FLYER";
    ok1(LXNavigation::NMEAv2::GenerateLXDT_PILOT_SET(pilot) == "LXDT,SET,PILOT,ACE,FLYER");
  }
  {
    NMEAInputLine line("ACE,FLYER");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_PILOT_ANS(line);
    ok1(parsing_result.name == "ACE");
    ok1(parsing_result.surname == "FLYER");
  }
}

static void
TestLXDT_TSK_PAR()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_TSK_PAR_GET() == "LXDT,GET,TSK_PAR");
  {
    LXNavigation::TaskParameters task;
    task.finish_1000 = true;
    task.finish_alt_offset = 5;
    task.aat_time.SetASCII("02:30");
    ok1(LXNavigation::NMEAv2::GenerateLXDT_TSK_PAR_SET(task) == "LXDT,SET,TSK_PAR,1,5,02:30");
  }
  {
    NMEAInputLine line("1,700,02:30");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_TSK_PAR_ANS(line);
    ok1(parsing_result);
    ok1(parsing_result->finish_1000);
    ok1(parsing_result->finish_alt_offset == 700);
    ok1(parsing_result->aat_time == "02:30");
  }
}

static void
TestLXDT_MC_BAL()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_MC_BAL_GET() == "LXDT,GET,MC_BAL");
  {
    LXNavigation::GlideParameters glider_parameters;
    LXNavigation::DeviceParameters device_parameters;
    glider_parameters.mac_cready = 1.1;
    glider_parameters.load_factor = 1.44;
    glider_parameters.bugs = 30;
    device_parameters.brightness = 55;
    device_parameters.vario_vol = 70;
    device_parameters.sc_vol = 20;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_MC_BAL_SET(std::make_pair(glider_parameters, device_parameters)) == "LXDT,SET,MC_BAL,1.1,1.44,30,55,70,20");
  }
  {
    NMEAInputLine line("1.1,1.44,30,55,70,20");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_MC_BAL_ANS(line);
    ok1(parsing_result.first.mac_cready);
    ok1(equals(*parsing_result.first.mac_cready, 1.1));
    ok1(parsing_result.first.load_factor);
    ok1(equals(*parsing_result.first.load_factor, 1.44));
    ok1(parsing_result.first.bugs);
    ok1(*parsing_result.first.bugs == 30);
    ok1(parsing_result.second.brightness == 55);
    ok1(parsing_result.second.vario_vol == 70);
    ok1(parsing_result.second.sc_vol == 20);
  }
}

static void
TestLXDT_RADIO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_RADIO_GET() == "LXDT,GET,RADIO");
  {
    LXNavigation::RadioParameters radio;
    radio.active_freq = 118.475;
    radio.standby_freq = 121.500;
    radio.volume = 9;
    radio.squelch = 8;
    radio.vox = 7;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_RADIO_SET(radio) == "LXDT,SET,RADIO,118.475,121.500,9,8,7");
  }
  {
    NMEAInputLine line("128.800,118.475,10,5,33");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_RADIO_ANS(line);
    ok1(parsing_result);
    ok1(equals(parsing_result->active_freq, 128.800));
    ok1(equals(parsing_result->standby_freq, 118.475));
    ok1(parsing_result->volume == 10);
    ok1(parsing_result->squelch == 5);
    ok1(parsing_result->vox == 33);
  }
  {
    NMEAInputLine line("128.800,11f8.475,10,5,33");
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_RADIO_ANS(line);
    ok1(!parsing_result);
  }
}

static void
TestLXDT_R_SWITCH()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_R_SWITCH_TOGGLE() == "LXDT,SET,R_SWITCH");
}

static void
TestLXDT_R_DUAL()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_R_DUAL_SET(true) == "LXDT,SET,R_DUAL,1");
  ok1(LXNavigation::NMEAv2::GenerateLXDT_R_DUAL_SET(false) == "LXDT,SET,R_DUAL,0");
}

static void
TestLXDT_R_SPACING()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_R_SPACING_SET(true) == "LXDT,SET,R_SPACING,1");
  ok1(LXNavigation::NMEAv2::GenerateLXDT_R_SPACING_SET(false) == "LXDT,SET,R_SPACING,0");
}

static void
TestLXDT_FLIGHTS_NO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_FLIGHTS_NO_GET() == "LXDT,GET,FLIGHTS_NO");
  NMEAInputLine line("9");
  auto result = LXNavigation::NMEAv2::ParseLXDT_FLIGHTS_NO_ANS(line);
  ok1(result);
  ok1(*result == 9);
}

static void
TestLXDT_FLIGHT_INFO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_FLIGHT_INFO_GET(3) == "LXDT,GET,FLIGHT_INFO,3");
  NMEAInputLine line("1,03JLQYT1,19.03.2020"
                     ",07:08:24,07:11:27,ACE,FLYER,D-KLXD,XD,0,10"
                     ",1260,98");
  auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_FLIGHT_INFO_ANS(line);
  ok1(parsing_result);
  ok1(parsing_result->flight_id == 1);
  ok1(parsing_result->filename == "03JLQYT1");
  ok1(parsing_result->date == BrokenDate(2020, 3, 19));
  ok1(parsing_result->take_off == BrokenTime(7, 8, 24));
  ok1(parsing_result->landing == BrokenTime(7, 11, 27));
  ok1(parsing_result->pilot_name == "ACE");
  ok1(parsing_result->pilot_surname == "FLYER");
  ok1(parsing_result->reg_no == "D-KLXD");
  ok1(parsing_result->comp_id == "XD");
  ok1(equals(parsing_result->min_gforce, 0));
  ok1(equals(parsing_result->max_gforce, 10));
  ok1(parsing_result->max_alt == 1260);
  ok1(equals(parsing_result->max_ias, 98));
}

static void
TestBinary_Common()
{
  {
    LXNavigation::Binary::PacketBuffer buffer({0x06});
    ok1(LXNavigation::Binary::ParseResult(buffer));
  }
  {
    LXNavigation::Binary::PacketBuffer buffer({0x15});
    ok1(!LXNavigation::Binary::ParseResult(buffer));
  }
}

static void
TestBinary_Task()
{
  {
    LXNavigation::Binary::PacketBuffer buffer
    { 0x06, // ACK
      0x50, 0x00, //flight id - 80
      0x31, 0x33, 0x43, 0x4C, 0x51, 0x56, 0x32, 0x32, 0x00, 0x00, //filename 13CLQV22
      0x96, 0x86, 0x25, 0x00, // date Julian Day
      0xd1, 0x8e, 0x00, 0x00, // time 14:54:06
      0xd1, 0x8e, 0x00, 0x00, // landing time 14:54:06
      0x45, 0x4d, 0x50, 0x54, 0x59, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, // pilot name EMPTY
      0x45, 0x4d, 0x50, 0x54, 0x59, 0x31, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, // pilot surname EMPTY1
      0x55, 0x52, 0x2d, 0x56, 0x59, 0x00, 0x11, 0x11, // registration number UR-VYAA
      0x41, 0x41, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, // competition ID AA
      0x0A, // Min G-force 1.0
      0x20, // Max G-force 3.2
      0x13, 0x1A, // Max altitude 6675
      0x90, 0x00, // Max IAS 144

      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
      0xAF, 0x33, 0x00, 0x00, // flight file size - 44851
      0x00 // CRC
    };
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(flight_info);
    ok1(flight_info->filename == "13CLQV22");
    ok1(flight_info->date == BrokenDate(2021, 03, 12));
    ok1(flight_info->take_off == BrokenTime(14, 54, 06));
    ok1(flight_info->landing == BrokenTime(14, 54, 06));
    ok1(flight_info->pilot_name == "EMPTY");
    ok1(flight_info->pilot_surname == "EMPTY1");
    ok1(flight_info->reg_no == "UR-VYAA");
    ok1(flight_info->comp_id == "AA");
    ok1(equals(flight_info->min_gforce, 1.0));
    ok1(equals(flight_info->max_gforce, 3.2));
    ok1(flight_info->max_alt == 6675);
    ok1(flight_info->max_ias == 144);
  }

  {// NACK
    LXNavigation::Binary::PacketBuffer buffer{ 0x15 };
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }

  {// incorrect CRC
    LXNavigation::Binary::PacketBuffer buffer;
    buffer.resize(92, 0);
    buffer[0] = 0x06;
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }

  {// size mismatch
    LXNavigation::Binary::PacketBuffer buffer(50, 0);
    buffer[0] = 0x06;
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);

    buffer.resize(99, 0);
    buffer[0] = 0x06;
    flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }
}

static void
TestBinary_Flight()
{
  {
    LXNavigation::Binary::FlightBlockBuffer expected_buffer{0x01, 0x02, 0x03, 0x04, 0x05};
    LXNavigation::Binary::FlightBlockBuffer buffer{0x06, 0x05, 0x00, 0x00, 0x00};
    buffer.insert(std::end(buffer), std::begin(expected_buffer), std::end(expected_buffer));
    buffer.push_back(0x00); // CRC

    LXNavigation::Binary::FlightBlockBuffer block_buffer;
    LXNavigation::Binary::ParseFlightBlock(buffer, block_buffer);

    ok1(std::make_pair(std::end(expected_buffer), std::end(block_buffer)) == std::mismatch(std::begin(expected_buffer), std::end(expected_buffer), std::begin(block_buffer), std::end(block_buffer)));
  }
  { // NACK
    LXNavigation::Binary::FlightBlockBuffer block_buffer(10, 11);
    LXNavigation::Binary::ParseFlightBlock({0x15}, block_buffer);

    ok1(block_buffer.empty());
  }
  { // incorrect CRC
    LXNavigation::Binary::FlightBlockBuffer buffer(11, 0);
    buffer[0] = 0x06;
    buffer[1] = 0x05;
    LXNavigation::Binary::FlightBlockBuffer block_buffer(11, 11);
    LXNavigation::Binary::ParseFlightBlock(buffer, block_buffer);

    ok1(block_buffer.empty());
  }
  { // size mismatch
    LXNavigation::Binary::FlightBlockBuffer buffer(7, 0);
    buffer[0] = 0x06;
    buffer[1] = 0x0A;
    buffer[6] = 0x00; //CRC
    LXNavigation::Binary::FlightBlockBuffer block_buffer(10, 11);
    LXNavigation::Binary::ParseFlightBlock({0x15}, block_buffer);

    ok1(block_buffer.empty());

    buffer[1] = 0x02;
    buffer[6] = 0x00; //CRC
    LXNavigation::Binary::ParseFlightBlock({0x15}, block_buffer);
    ok1(block_buffer.empty());
  }

  {
    LXNavigation::Binary::PacketBuffer buffer;
    LXNavigation::Binary::GenerateFlightBlockGet(9, 1, buffer);
    LXNavigation::Binary::PacketBuffer expected_buffer{0x02, 0xF1, 0x09, 0x00, 0x01, 0x00, 0xC8};

    ok1(std::make_pair(std::end(expected_buffer), std::end(buffer)) == std::mismatch(std::begin(expected_buffer), std::end(expected_buffer), std::begin(buffer), std::end(buffer)));
  }
  {
    LXNavigation::Binary::PacketBuffer buffer;
    LXNavigation::Binary::GenerateFlightInfoGet(1, buffer);
    LXNavigation::Binary::PacketBuffer expected_buffer{0x02, 0xF0, 0x01, 0x00, 0x3D};

    ok1(std::make_pair(std::end(expected_buffer), std::end(buffer)) == std::mismatch(std::begin(expected_buffer), std::end(expected_buffer), std::begin(buffer), std::end(buffer)));
  }

  {
    LXNavigation::Binary::PacketBuffer buffer
    { 0x06, // ACK
      0x50, 0x00, //flight id - 80
      0x31, 0x33, 0x43, 0x4C, 0x51, 0x56, 0x32, 0x32, 0x00, 0x00, //filename 13CLQV22
      0x96, 0x86, 0x25, 0x00, // date Julian Day
      0xd1, 0x8e, 0x00, 0x00, // time 14:54:06
      0xd1, 0x8e, 0x00, 0x00, // landing time 14:54:06
      0x45, 0x4d, 0x50, 0x54, 0x59, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, // pilot name EMPTY
      0x45, 0x4d, 0x50, 0x54, 0x59, 0x31, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, // pilot surname EMPTY1
      0x55, 0x52, 0x2d, 0x56, 0x59, 0x00, 0x11, 0x11, // registration number UR-VYAA
      0x41, 0x41, 0x00, 0x11, 0x11, 0x11, 0x11, 0x11, // competition ID AA
      0x0A, // Min G-force 1.0
      0x20, // Max G-force 3.2
      0x13, 0x1A, // Max altitude 6675
      0x90, 0x00, // Max IAS 144

      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
      0xAF, 0x33, 0x00, 0x00, // flight file size - 44851
      0x00 // CRC
    };
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(flight_info);
    ok1(flight_info->filename == "13CLQV22");
    ok1(flight_info->date == BrokenDate(2021, 03, 12));
    ok1(flight_info->take_off == BrokenTime(14, 54, 06));
    ok1(flight_info->landing == BrokenTime(14, 54, 06));
    ok1(flight_info->pilot_name == "EMPTY");
    ok1(flight_info->pilot_surname == "EMPTY1");
    ok1(flight_info->reg_no == "UR-VYAA");
    ok1(flight_info->comp_id == "AA");
    ok1(equals(flight_info->min_gforce, 1.0));
    ok1(equals(flight_info->max_gforce, 3.2));
    ok1(flight_info->max_alt == 6675);
    ok1(flight_info->max_ias == 144);
  }

  {// NACK
    LXNavigation::Binary::PacketBuffer buffer{ 0x15 };
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }

  {// incorrect CRC
    LXNavigation::Binary::PacketBuffer buffer;
    buffer.resize(92, 0);
    buffer[0] = 0x06;
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }

  {// size mismatch
    LXNavigation::Binary::PacketBuffer buffer(50, 0);
    buffer[0] = 0x06;
    auto flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);

    buffer.resize(99, 0);
    buffer[0] = 0x06;
    flight_info = LXNavigation::Binary::ParseFlightInfo(buffer);
    ok1(!flight_info);
  }
}

static void
TestBinary_Zone()
{
}

static void
TestBinary_Class()
{
}

static void
TestBinary_Radio()
{
}

int main(int argc, char **argv)
{
  plan_tests(131);

  TestPFLX0();
  TestPFLX2();
  TestLXWP0();
  TestLXWP1();
  TestLXWP2();
  TestLXWP3();

  TestLXBC();
  TestLXDT_ANS_Status();
  TestLXDT_INFO();
  TestLXDT_TP();
  TestLXDT_ZONE();
  TestLXDT_GLIDER();
  TestLXDT_PILOT();
  TestLXDT_TSK_PAR();
  TestLXDT_MC_BAL();
  TestLXDT_RADIO();
  TestLXDT_R_SWITCH();
  TestLXDT_R_DUAL();
  TestLXDT_R_SPACING();
  TestLXDT_FLIGHTS_NO();
  TestLXDT_FLIGHT_INFO();

  TestBinary_Common();
  TestBinary_Task();
  TestBinary_Flight();
  TestBinary_Zone();
  TestBinary_Class();
  TestBinary_Radio();
  return exit_status();
}

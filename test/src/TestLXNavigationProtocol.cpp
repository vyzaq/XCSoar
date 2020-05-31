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
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line("1.5,1.11,13,2.96,-3.03,1.35,45");
    LXNavigation::NMEAv1::ParseLXWP2(line, info);
    ok1(info.settings.mac_cready_available);
    ok1(equals(info.settings.mac_cready, 1.5));
    ok1(info.settings.ballast_overload_available);
    ok1(equals(info.settings.ballast_overload, 1.11));
    ok1(info.settings.bugs_available);
    ok1(equals(info.settings.bugs, 0.87));
    ok1(info.settings.volume_available);
    ok1(info.settings.volume == 45);
  }
  {
    NMEAInfo info;
    info.Reset();
    info.clock = 1;
    info.alive.Update(info.clock);

    NMEAInputLine line(",,,,,,");
    LXNavigation::NMEAv1::ParseLXWP2(line, info);
    ok1(!info.settings.mac_cready_available);
    ok1(!info.settings.ballast_overload_available);
    ok1(!info.settings.bugs_available);
    ok1(!info.settings.volume_available);
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
    ok1(equals(info.settings.qnh.GetHectoPascal(), 990));
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
  ok1(LXNavigation::NMEAv2::ParseLXDT_ANS_Status("ANS,OK").status == LXNavigation::Status::Ok);
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ANS_Status("ANS,ERROR,Parameter count mismatch");
    ok1(parsing_result.status == LXNavigation::Status::Error);
    ok1(parsing_result.description == "Parameter count mismatch");
  }
}

static void
TestLXDT_INFO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_INFO_GET() == "LXDT,GET,INFO");
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_INFO_ANS("ANS,INFO,LX Era,34949,1.4,1.1,0-[0],00,Empty,Empty");
    ok1(parsing_result.name == "LX Era");
    ok1(parsing_result.serial == 34949);
    ok1(parsing_result.sw_version == 1.4);
    ok1(parsing_result.hw_version == 1.1);
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
    turnpoint.location = GeoPoint(Angle::Degrees(45.810283), Angle::Degrees(15.1127));
    turnpoint.total_tp_count = 5;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_TP_SET(turnpoint) == "LXDT,SET,TP,0,5,2748617,906762,NOVO MESTO");
  }
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_TP_ANS("2,2,2748617,906762,NOVO MESTO ");
    ok1(parsing_result.id == 2);
    ok1(parsing_result.name == "NOVO MESTO ");
    ok1(parsing_result.location == GeoPoint(Angle::Degrees(45.810283), Angle::Degrees(15.1127)));
    ok1(parsing_result.type == LXNavigation::TurnpointType::Landing);
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
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_ZONE_ANS("2,3,0,1,90,60,309,5000,3500,174");
    ok1(parsing_result.id == 2);
    ok1(parsing_result.direction == LXNavigation::Direction::ToPrevious);
    ok1(parsing_result.is_auto_next == false);
    ok1(parsing_result.is_line);
    ok1(parsing_result.a1 == 90);
    ok1(parsing_result.a2 == 60);
    ok1(parsing_result.a21 == 309);
    ok1(parsing_result.r1 == 5000);
    ok1(parsing_result.r2 == 3500);
    ok1(parsing_result.elevation == 174);
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
    ok1(LXNavigation::NMEAv2::GenerateLXDT_GLIDER_SET(glider) == "LXDT,SET,GLIDER,D-KLXD,XD,OPEN");
  }
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_GLIDER_ANS("JS3 15m,D-KLXD,XD,OPEN");
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
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_PILOT_ANS("ACE,FLYER");
    ok1(parsing_result.name == "ACE");
    ok1(parsing_result.surname == "FLYER");
  }
}

static void
TestLXDT_TSK_PAR()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_TSK_PAR_GET() == "GET,TSK_PAR");
  {
    LXNavigation::TaskParameters task;
    task.finish_1000 = true;
    task.finish_alt_offset = 0;
    task.aat_time_sec = 9000;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_TSK_PAR_SET(task) == "SET,TSK_PAR,1,,02:30");
  }
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_TSK_PAR_ANS("1,700,02:30");
    ok1(parsing_result.finish_1000);
    ok1(parsing_result.finish_alt_offset == 700);
    ok1(equals(parsing_result.aat_time_sec, 9000));
  }
}

static void
TestLXDT_MC_BAL()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_MC_BAL_GET() == "LXDT,GET,MC_BAL");
  {
    LXNavigation::GlideParameters glider_parameters;
    LXNavigation::DeviceParameters device_parameters;
    glider_parameters.mc_ready =1.1;
    glider_parameters.load_factor = 200;
    glider_parameters.bugs = 30;
    device_parameters.brightness = 55;
    device_parameters.vario_vol = 70;
    device_parameters.sc_vol = 20;
    ok1(LXNavigation::NMEAv2::GenerateLXDT_MC_BAL_SET(std::make_pair(glider_parameters, device_parameters)) == "SET,MC_BAL,1.1,200,30,55,70,20");
  }
  {
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_MC_BAL_ANS("1.1,200,30,55,70,20");
    ok1(equals(parsing_result.first.mc_ready, 1.1));
    ok1(parsing_result.first.load_factor == 200);
    ok1(parsing_result.first.bugs == 30);
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
    auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_RADIO_ANS("128.800,118.475,10,5,33");
    ok1(equals(parsing_result.active_freq, 128.800));
    ok1(equals(parsing_result.standby_freq, 118.475));
    ok1(parsing_result.volume == 10);
    ok1(parsing_result.squelch == 5);
    ok1(parsing_result.vox == 33);
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
  ok1(LXNavigation::NMEAv2::ParseLXDT_FLIGHTS_NO_ANS("9") == 9);
}

static void
TestLXDT_FLIGHT_INFO()
{
  ok1(LXNavigation::NMEAv2::GenerateLXDT_FLIGHT_INFO_GET(3) == "LXDT,GET,FLIGHT_INFO,3");
  auto parsing_result = LXNavigation::NMEAv2::ParseLXDT_FLIGHT_INFO_ANS("1,03JLQYT1,19.03.2020"
                                                               ",07:08:24,07:11:27,ACE,FLYER,D-KLXD,XD,0,10"
                                                               ",1260,98");
  ok1(parsing_result.flight_id == 1);
  ok1(parsing_result.filename == "03JLQYT1");
  ok1(parsing_result.date == BrokenDate(2020, 3, 19));
  ok1(parsing_result.take_off == BrokenTime(7, 8, 24));
  ok1(parsing_result.landing == BrokenTime(7, 11, 27));
  ok1(parsing_result.pilot_name == "ACE");
  ok1(parsing_result.pilot_surname == "FLYER");
  ok1(parsing_result.reg_no == "D-KLXD");
  ok1(parsing_result.comp_id == "XD");
  ok1(equals(parsing_result.min_gforce, 0));
  ok1(equals(parsing_result.max_gforce, 1));
  ok1(parsing_result.max_alt == 1298);
  ok1(equals(parsing_result.max_ias, 98));
}

int main(int argc, char **argv)
{
  plan_tests(84);

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

  return exit_status();
}

#include "NMEAv2Protocol.hpp"
#include "NMEAInfoConvertors.hpp"

#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"

namespace LXNavigation
{
namespace NMEAv2
{
namespace
{
const char* GetName(GliderClass glider_class )
{
  switch(glider_class)
  {
  case GliderClass::Open:
    return "OPEN";
  case GliderClass::World:
    return "WORLD";
  case GliderClass::Double:
    return "DOUBLE";
  case GliderClass::Meter15:
    return "METER15";
  case GliderClass::Meter18:
    return "METER18";
  case GliderClass::MotorGL:
    return "MOTOR_GL";
  case GliderClass::Standard:
    return "STANDARD";
  default:
    return "";
  }
}

template<typename String>
GliderClass GetGliderClass(const String& class_name)
{
  if(class_name == "OPEN")
    return GliderClass::Open;
  if(class_name == "WORLD")
    return GliderClass::World;
  if(class_name == "DOUBLE")
    return GliderClass::Double;
  if(class_name == "METER15")
    return GliderClass::Meter15;
  if(class_name == "METER18")
    return GliderClass::Meter18;
  if(class_name == "MOTOR_GL")
    return GliderClass::MotorGL;
  if(class_name == "STANDARD")
    return GliderClass::Standard;
  return GliderClass::None;
}
}

Message ParseLXDT_ANS_Status(NMEAInputLine &line)
{
  Message result;
  line.Read(result.buffer(), result.capacity());
  return result;
}

Message GenerateLXDT_INFO_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,INFO");
  return result;
}

DeviceInfo ParseLXDT_INFO_ANS(NMEAInputLine &line)
{
  DeviceInfo result;
  line.Read(result.name.buffer(), result.name.capacity());
  result.serial = line.Read(0);
  result.sw_version = line.Read(0.0);
  result.hw_version = line.Read(0.0);
  return result;
}

Message GenerateLXDT_TP_GET(u_int16_t turnpoint_id)
{
  Message result;
  result.Format("LXDT,GET,TP,%d", turnpoint_id);
  return result;
}

Message GenerateLXDT_TP_SET(const TurnpointData& data)
{
  Message result;
  int32_t latitude = data.location.latitude.AbsoluteDegrees()*60000;
  int32_t longitude = data.location.longitude.AbsoluteDegrees()*60000;
  result.Format("LXDT,SET,TP,%d,%d,%d,%d,%s",
                data.id, data.total_tp_count, latitude, longitude, data.name.c_str());
  return result;
}
std::optional<TurnpointData> ParseLXDT_TP_ANS(NMEAInputLine &line)
{
  TurnpointData result{};
  int32_t latitude = 0;
  int32_t longitude = 0;
  int type = 0;
  unsigned id;
  if(!line.ReadChecked(id))
    return {};
  result.id = id;
  if(!line.ReadChecked(type))
    return {};
  result.type = static_cast<TurnpointType>(type);
  if(!line.ReadChecked(latitude))
    return {};
  if(!line.ReadChecked(longitude))
    return {};
  result.location = GeoPoint(Angle::Degrees(longitude / 60000.0), Angle::Degrees(latitude / 60000.0));
  line.Read(result.name.buffer(), result.name.capacity());
  if(result.name.empty())
    return {};
  return result;
}

Message GenerateLXDT_ZONE_GET(u_int16_t turnpoint_id)
{
  Message result;
  result.Format("LXDT,GET,ZONE,%d", turnpoint_id);
  return result;
}

Message GenerateLXDT_ZONE_SET(const TurnpointZone& data)
{
  Message result;
  result.Format("LXDT,SET,ZONE,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                data.id, data.direction, data.is_auto_next ? 1 : 0, data.is_line ? 1 : 0,
                data.a1, data.a2, data.a21, data.r1, data.r2, data.elevation);
  return result;
}

std::optional<TurnpointZone> ParseLXDT_ZONE_ANS(NMEAInputLine &line)
{
  TurnpointZone result{};
  unsigned value;
  if(!line.ReadChecked(value))
    return {};
  result.id = value;
  if(!line.ReadChecked(value))
    return {};
  result.direction = static_cast<Direction>(value);
  if(!line.ReadChecked(value))
    return {};
  result.is_auto_next = value;
  if(!line.ReadChecked(value))
    return {};
  result.is_line = value;
  if(!line.ReadChecked(value))
    return {};
  result.a1 = value;
  if(!line.ReadChecked(value))
    return {};
  result.a2 = value;
  if(!line.ReadChecked(value))
    return {};
  result.a21 = value;
  if(!line.ReadChecked(value))
    return {};
  result.r1 = value;
  if(!line.ReadChecked(value))
    return {};
  result.r2 = value;
  if(!line.ReadChecked(value))
    return {};
  result.elevation = value;
  return result;
}

Message GenerateLXDT_GLIDER_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,GLIDER");
  return result;
}

Message GenerateLXDT_GLIDER_SET(const GliderInfo& data)
{
  Message result;
  result.Format("LXDT,SET,GLIDER,%s,%s,%s",
                data.reg_no.c_str(), data.comp_id.c_str(), GetName(data.glider_class));
  return result;
}

GliderInfo ParseLXDT_GLIDER_ANS(NMEAInputLine &line)
{
  NarrowString<10> glider_class_name;
  GliderInfo result{};
  line.Read(result.polar_name.buffer(), result.polar_name.capacity());
  line.Read(result.reg_no.buffer(), result.reg_no.capacity());
  line.Read(result.comp_id.buffer(), result.comp_id.capacity());
  line.Read(glider_class_name.buffer(), glider_class_name.capacity());
  result.glider_class = GetGliderClass(glider_class_name);
  return result;
}

Message GenerateLXDT_PILOT_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,PILOT");
  return result;
}

Message GenerateLXDT_PILOT_SET(const PilotInfo& data)
{
  Message result;
  result.Format("LXDT,SET,PILOT,%s,%s", data.name.c_str(), data.surname.c_str());
  return result;
}

PilotInfo ParseLXDT_PILOT_ANS(NMEAInputLine &line)
{
  PilotInfo result{};
  line.Read(result.name.buffer(), result.name.capacity());
  line.Read(result.surname.buffer(), result.surname.capacity());
  return result;
}

Message GenerateLXDT_TSK_PAR_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,TSK_PAR");
  return result;
}

Message GenerateLXDT_TSK_PAR_SET(const TaskParameters& data)
{
  Message result;
  result.Format("LXDT,SET,TSK_PAR,%d,%d,%s",
                data.finish_1000, data.finish_alt_offset, data.aat_time.c_str());
  return result;
}

std::optional<TaskParameters> ParseLXDT_TSK_PAR_ANS(NMEAInputLine &line)
{
  TaskParameters result;
  unsigned value;
  if(!line.ReadChecked(value))
    return {};
  result.finish_1000 = value;
  if(!line.ReadChecked(value))
    return {};
  result.finish_alt_offset = value;
  line.Read(result.aat_time.buffer(), result.aat_time.capacity());
  return result;
}

Message GenerateLXDT_MC_BAL_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,MC_BAL");
  return result;
}

Message GenerateLXDT_MC_BAL_SET(const std::pair<GlideParameters, DeviceParameters>& data)
{
  Message result;
  NarrowString<5> mc("");
  NarrowString<5> ballast("");
  NarrowString<4> bugs("");
  NarrowString<4> brightness("");
  NarrowString<4> vario_vol("");
  NarrowString<4> sc_vol("");
  if(data.first.mac_cready)
    mc.Format("%.1f", *data.first.mac_cready);
  if(data.first.load_factor)
    ballast.Format("%.2f", *data.first.load_factor);
  if(data.first.bugs)
    bugs.Format("%d", *data.first.bugs);
  if(data.second.brightness)
    brightness.Format("%d", *data.second.brightness);
  if(data.second.vario_vol)
    vario_vol.Format("%d", *data.second.vario_vol);
  if(data.second.sc_vol)
    sc_vol.Format("%d", *data.second.sc_vol);
  result.Format("LXDT,SET,MC_BAL,%s,%s,%s,%s,%s,%s",
                mc.c_str(), ballast.c_str(), bugs.c_str(),
                brightness.c_str(), vario_vol.c_str(), sc_vol.c_str());
  return result;
}
std::pair<GlideParameters, DeviceParameters> ParseLXDT_MC_BAL_ANS(NMEAInputLine &line)
{
  GlideParameters glide_parameters;
  DeviceParameters device_parameters;


  ReadElement(line, glide_parameters.mac_cready);
  ReadElement(line, glide_parameters.load_factor);
  ReadElement(line, glide_parameters.bugs);

  ReadElement(line, device_parameters.brightness);
  ReadElement(line, device_parameters.vario_vol);
  ReadElement(line, device_parameters.sc_vol);
  return {glide_parameters, device_parameters};
}

Message GenerateLXDT_RADIO_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,RADIO");
  return result;
}
Message GenerateLXDT_RADIO_SET(const RadioParameters& data)
{
  Message result;
  result.Format("LXDT,SET,RADIO,%.3f,%.3f,%d,%d,%d",
                data.active_freq, data.standby_freq, data.volume, data.squelch, data.vox);
  return result;
}
std::optional<RadioParameters> ParseLXDT_RADIO_ANS(NMEAInputLine &line)
{
  RadioParameters result;
  if(!line.ReadChecked(result.active_freq))
    return {};
  if(!line.ReadChecked(result.standby_freq))
    return {};
  if(!line.ReadChecked(result.volume))
    return {};
  if(!line.ReadChecked(result.squelch))
    return {};
  if(!line.ReadChecked(result.vox))
    return {};
  return result;
}

Message GenerateLXDT_FLIGHTS_NO_GET()
{
  Message result;
  result.SetASCII("LXDT,GET,FLIGHTS_NO");
  return result;
}

std::optional<uint16_t> ParseLXDT_FLIGHTS_NO_ANS(NMEAInputLine &line)
{
  unsigned result;
  if(!line.ReadChecked(result))
    return {};
  return result;
}

Message GenerateLXDT_FLIGHT_INFO_GET(uint16_t flight_index)
{
  Message result;
  result.Format("LXDT,GET,FLIGHT_INFO,%d", flight_index);
  return result;
}

std::optional<FlightInfo> ParseLXDT_FLIGHT_INFO_ANS(NMEAInputLine &line)
{
  FlightInfo result;
  NarrowString<32> buffer;
  unsigned value;
  if(!line.ReadChecked(value))
    return {};
  result.flight_id = value;
  line.Read(result.filename.buffer(), result.filename.capacity());
  { //date
    line.Read(buffer.buffer(), buffer.capacity());
    unsigned day, month, year;
    sscanf(buffer.c_str(), "%02u.%02u.%04u", &day, &month, &year);
    result.date = BrokenDate(year, month, day);
    if(!result.date.IsPlausible())
      return {};
  }
  {
    unsigned hours, minutes, seconds;
    line.Read(buffer.buffer(), buffer.capacity()); //takeoff
    sscanf(buffer.c_str(), "%02u:%02u:%02u", &hours, &minutes, &seconds);
    result.take_off = BrokenTime(hours, minutes, seconds);
    if(!result.take_off.IsPlausible())
      return {};
    line.Read(buffer.buffer(), buffer.capacity()); //landing
    sscanf(buffer.c_str(), "%02u:%02u:%02u", &hours, &minutes, &seconds);
    result.landing = BrokenTime(hours, minutes, seconds);
    if(!result.landing.IsPlausible())
      return {};
  }
  line.Read(result.pilot_name.buffer(), result.pilot_name.capacity());
  line.Read(result.pilot_surname.buffer(), result.pilot_surname.capacity());
  line.Read(result.reg_no.buffer(), result.reg_no.capacity());
  line.Read(result.comp_id.buffer(), result.comp_id.capacity());

  {
    int value;
    if(!line.ReadChecked(value))
      return {};
    result.min_gforce = value;
    if(!line.ReadChecked(value))
      return {};
    result.max_gforce = value;
  }
  if(!line.ReadChecked(value))
    return {};
  result.max_alt = value;
  if(!line.ReadChecked(value))
    return {};
  result.max_ias = value;
  return result;
}

Message GenerateLXDT_R_SWITCH_TOGGLE()
{
  Message result;
  result.SetASCII("LXDT,SET,R_SWITCH");
  return result;
}
Message GenerateLXDT_R_DUAL_SET(bool is_dual_enabled)
{
  Message result;
  result.Format("LXDT,SET,R_DUAL,%d", is_dual_enabled ? 1 : 0);
  return result;
}
Message GenerateLXDT_R_SPACING_SET(bool is_833)
{
  Message result;
  result.Format("LXDT,SET,R_SPACING,%d", is_833 ? 1 : 0);
  return result;
}

}
}

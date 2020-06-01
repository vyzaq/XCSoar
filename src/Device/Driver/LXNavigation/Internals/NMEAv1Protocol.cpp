#include "NMEAv1Protocol.hpp"
#include "NMEAInfoConvertors.hpp"

#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"
#include "Units/System.hpp"
#include "Engine/GlideSolvers/PolarCoefficients.hpp"

namespace LXNavigation
{
namespace NMEAv1
{
const char* FLIGHT_DATA_PARAMETERS = "LXWP0";
const char* BASIC_DEVICE_INFO = "LXWP1";
const char* BASIC_GLIDE_INFO_PARAMETERS = "LXWP2";
const char* ADVANCED_GLIDE_INFO_PARAMETERS = "LXWP3";

Message GeneratePFLX0(const PFLX0Request &request)
{
  if(request.empty() || request.size() > 4)
    return {};
  Message result("PFLX0");
  for(const auto& sentence : request)
  {
    Message sentence_string;
    sentence_string.Format(",%s,%d", sentence.first.c_str(), sentence.second);
    result += sentence_string;
  }
  return result;
}

Message GeneratePFLX2ForMcReady(double mc)
{
  Message result;
  result.Format("PFLX2,%1.1f,,,,,,", mc);
  return result;
}

Message GeneratePFLX2ForBugs(uint16_t bugs)
{
  Message result;
  result.Format("PFLX2,,,%u,,,,", bugs);
  return result;
}

Message GeneratePFLX2ForVolume(unsigned volume)
{
  Message result;
  result.Format("PFLX2,,,,,,,%u", volume);
  return result;
}

Message GeneratePFLX2ForBallast(double fraction, double overload)
{
  Message result;
  result.Format("PFLX2,,%.2f,,,,,", overload);
  return result;
}

Message GeneratePFLX2ForPolar(const PolarCoefficients &polar)
{
  Message result;
  result.Format("PFLX2,,,,%.2f,%.2f,%.2f,", polar.a, polar.b, polar.c);
  return result;
}

void ParseLXWP0(NMEAInputLine &line, NMEAInfo &info)
{
  line.Skip();
  double airspeed;
  bool tas_available = line.ReadChecked(airspeed);

  double altitude;
  bool altitude_available = line.ReadChecked(altitude);
  if (altitude_available)
    info.ProvideBaroAltitudeTrue(altitude);
  if(tas_available) {
    if(altitude_available)
      info.ProvideTrueAirspeedWithAltitude(Units::ToSysUnit(airspeed, Unit::KILOMETER_PER_HOUR), altitude);
    else
      info.ProvideTrueAirspeed(Units::ToSysUnit(airspeed, Unit::KILOMETER_PER_HOUR));
  }

  double vario;
  if (line.ReadChecked(vario))
    info.ProvideTotalEnergyVario(vario);

  line.Skip(5);

  int magnetic_heading;
  bool heading_valid = line.ReadChecked(magnetic_heading);
  if(heading_valid) {
    info.heading_available.Update(info.clock);
    info.heading = Angle::Degrees(magnetic_heading);
  }

  int wind_direction = line.Read(0);
  double wind_speed = line.Read(0.0);
  SpeedVector wind_vector(Angle::Degrees(wind_direction), Units::ToSysUnit(wind_speed, Unit::KILOMETER_PER_HOUR));
  info.ProvideExternalWind(wind_vector);
}

DeviceInfo ParseLXWP1(NMEAInputLine &line)
{
  DeviceInfo result{};
  line.Read(result.name.buffer(), result.name.capacity());
  result.serial = line.Read(0);
  result.sw_version = line.Read(0.0);
  result.hw_version = line.Read(0.0);
  return result;
}

std::tuple<GlideParameters, PolarCoefficients, int> ParseLXWP2(NMEAInputLine &line)
{
  GlideParameters glide_parameters;
  PolarCoefficients polar{};

  ReadElement(line, glide_parameters.mc_ready);
  ReadElement(line, glide_parameters.load_factor);
  ReadElement(line, glide_parameters.bugs);

  line.ReadChecked(polar.a);
  line.ReadChecked(polar.b);
  line.ReadChecked(polar.c);

  int volume = -1;
  line.ReadChecked(volume);

  return {glide_parameters, polar, volume};
}

void ParseLXWP3(NMEAInputLine &line, NMEAInfo &info)
{

}

}
}

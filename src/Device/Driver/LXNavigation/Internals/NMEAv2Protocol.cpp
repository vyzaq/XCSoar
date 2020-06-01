#include "NMEAv2Protocol.hpp"
#include "NMEAInfoConvertors.hpp"

#include "NMEA/InputLine.hpp"
#include "NMEA/Info.hpp"

namespace LXNavigation
{
namespace NMEAv2
{

StatusResult ParseLXDT_ANS_Status(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_INFO_GET()
{
  return {};
}

DeviceInfo ParseLXDT_INFO_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_TP_GET(u_int16_t turnpoint_id)
{
  return {};
}

Message GenerateLXDT_TP_SET(const TurnpointData& data)
{
  return {};
}
TurnpointData ParseLXDT_TP_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_ZONE_GET(u_int16_t turnpoint_id)
{
  return {};
}
Message GenerateLXDT_ZONE_SET(const TurnpointZone& data)
{
  return {};
}
TurnpointZone ParseLXDT_ZONE_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_GLIDER_GET()
{
  return {};
}
Message GenerateLXDT_GLIDER_SET(const GliderInfo& data)
{
  return {};
}
GliderInfo ParseLXDT_GLIDER_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_PILOT_GET()
{
  return {};
}
Message GenerateLXDT_PILOT_SET(const PilotInfo& data)
{
  return {};
}
PilotInfo ParseLXDT_PILOT_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_TSK_PAR_GET()
{
  return {};
}
Message GenerateLXDT_TSK_PAR_SET(const TaskParameters& data)
{
  return {};
}
TaskParameters ParseLXDT_TSK_PAR_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_MC_BAL_GET()
{
  return {};
}
Message GenerateLXDT_MC_BAL_SET(const std::pair<GlideParameters, DeviceParameters>& data)
{
  return {};
}
std::pair<GlideParameters, DeviceParameters> ParseLXDT_MC_BAL_ANS(NMEAInputLine &line)
{
  GlideParameters glide_parameters;
  DeviceParameters device_parameters;


  ReadElement(line, glide_parameters.mc_ready);
  ReadElement(line, glide_parameters.load_factor);
  ReadElement(line, glide_parameters.bugs);

  ReadElement(line, device_parameters.brightness);
  ReadElement(line, device_parameters.vario_vol);
  ReadElement(line, device_parameters.sc_vol);
  return {glide_parameters, device_parameters};
}

Message GenerateLXDT_RADIO_GET()
{
  return {};
}
Message GenerateLXDT_RADIO_SET(const RadioParameters& data)
{
  return {};
}
RadioParameters ParseLXDT_RADIO_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_FLIGHTS_NO_GET()
{
  return {};
}
int16_t ParseLXDT_FLIGHTS_NO_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_FLIGHT_INFO_GET(u_int16_t flight_index)
{
  return {};
}
FlightInfo ParseLXDT_FLIGHT_INFO_ANS(const NMEAInputLine &line)
{
  return {};
}

Message GenerateLXDT_R_SWITCH_TOGGLE()
{
  return {};
}
Message GenerateLXDT_R_DUAL_SET(bool is_dual_enabled)
{
  return {};
}
Message GenerateLXDT_R_SPACING_SET(bool is_833)
{
  return {};
}

}
}

#include "LXDT.hpp"

namespace LXNavigation
{

StatusResult ParseLXDT_ANS_Status(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_INFO_GET()
{
  return {};
}

DeviceInfo ParseLXDT_INFO_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_TP_GET(u_int16_t turnpoint_id)
{
  return {};
}

LXDTMessage GenerateLXDT_TP_SET(const TurnpointData& data)
{
  return {};
}
TurnpointData ParseLXDT_TP_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_ZONE_GET(u_int16_t turnpoint_id)
{
  return {};
}
LXDTMessage GenerateLXDT_ZONE_SET(const TurnpointZone& data)
{
  return {};
}
TurnpointZone ParseLXDT_ZONE_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_GLIDER_GET()
{
  return {};
}
LXDTMessage GenerateLXDT_GLIDER_SET(const GliderInfo& data)
{
  return {};
}
GliderInfo ParseLXDT_GLIDER_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_PILOT_GET()
{
  return {};
}
LXDTMessage GenerateLXDT_PILOT_SET(const PilotInfo& data)
{
  return {};
}
PilotInfo ParseLXDT_PILOT_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_TSK_PAR_GET()
{
  return {};
}
LXDTMessage GenerateLXDT_TSK_PAR_SET(const TaskParameters& data)
{
  return {};
}
TaskParameters ParseLXDT_TSK_PAR_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_MC_BAL_GET()
{
  return {};
}
LXDTMessage GenerateLXDT_MC_BAL_SET(const std::pair<GlideParameters, DeviceParameters>& data)
{
  return {};
}
std::pair<GlideParameters, DeviceParameters> ParseLXDT_MC_BAL_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_RADIO_GET()
{
  return {};
}
LXDTMessage GenerateLXDT_RADIO_SET(const RadioParameters& data)
{
  return {};
}
RadioParameters ParseLXDT_RADIO_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_FLIGHTS_NO_GET()
{
  return {};
}
int16_t ParseLXDT_FLIGHTS_NO_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_FLIGHT_INFO_GET(u_int16_t turnpoint_id)
{
  return {};
}
FlightInfo ParseLXDT_FLIGHT_INFO_ANS(const char *line)
{
  return {};
}

LXDTMessage GenerateLXDT_R_SWITCH_TOGGLE()
{
  return {};
}
LXDTMessage GenerateLXDT_R_DUAL_SET(bool is_dual_enabled)
{
  return {};
}
LXDTMessage GenerateLXDT_R_SPACING_SET(bool is_833)
{
  return {};
}

}

#include "NMEAInfoConvertors.hpp"
#include "NMEA/Info.hpp"

namespace LXNavigation
{

void ConvertToNMEAInfo(GlideParameters &parameters, NMEAInfo &info)
{
  if(parameters.mac_cready)
    info.settings.ProvideMacCready(*parameters.mac_cready, info.clock);
  if(parameters.load_factor)
    info.settings.ProvideBallastOverload(*parameters.load_factor, info.clock);
  if(parameters.bugs)
    info.settings.ProvideBugs((100 - *parameters.bugs) / 100., info.clock);
}

void ConvertToNMEAInfo(DeviceParameters &parameters, NMEAInfo &info)
{
}

void ConvertToNMEAInfo(PolarCoefficients &parameters, NMEAInfo &info)
{
}

}

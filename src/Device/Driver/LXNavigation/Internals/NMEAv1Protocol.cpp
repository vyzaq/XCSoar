#include "NMEAv1Protocol.hpp"

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
    return {};
}

Message GeneratePFLX2ForMcReady(double mc)
{
    return {};
}

Message GeneratePFLX2ForBugs(double bugs)
{
    return {};
}

Message GeneratePFLX2ForVolume(unsigned volume)
{
    return {};
}

Message GeneratePFLX2ForBallast(double fraction, double overload)
{
  return {};
}

bool IsLineMatch(const NMEAInputLine &nmea_line, Sentences sentence)
{
  return true;
}

void ParseLXWP0(const NMEAInputLine &line, NMEAInfo &info)
{

}

DeviceInfo ParseLXWP1(const NMEAInputLine &line)
{
  return {};
}

void ParseLXWP2(const NMEAInputLine &line, NMEAInfo &info)
{

}

void ParseLXWP3(const NMEAInputLine &line, NMEAInfo &info)
{

}

}
}

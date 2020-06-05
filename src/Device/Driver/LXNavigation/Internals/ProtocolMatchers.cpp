#include "ProtocolMatchers.hpp"
#include "NMEA/InputLine.hpp"

namespace LXNavigation
{
template<>
bool MatchSentence<Sentences::LXDT>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("$LXDT");
}

template<>
bool MatchSentence<Sentences::LXWP0>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("$LXWP0");
}

template<>
bool MatchSentence<Sentences::LXWP1>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("$LXWP1");
}

template<>
bool MatchSentence<Sentences::LXWP2>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("$LXWP2");
}

template<>
bool MatchSentence<Sentences::LXWP3>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("$LXWP3");
}

template<>
bool MatchCommand<NMEAv2::SentenceCode::INFO>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("INFO");
}

template<>
bool MatchCommand<NMEAv2::SentenceCode::MC_BAL>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("MC_BAL");
}

template<>
bool MatchCommand<NMEAv2::SentenceCode::OK>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("OK");
}

template<>
bool MatchCommand<NMEAv2::SentenceCode::ERROR>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("ERROR");
}

template<>
bool MatchStatus<NMEAv2::SentenceAction::ANS>(NMEAInputLine& nmea_line)
{
  return nmea_line.ReadCompare("ANS");
}
}

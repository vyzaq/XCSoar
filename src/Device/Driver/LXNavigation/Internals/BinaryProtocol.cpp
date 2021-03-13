#include "BinaryProtocol.hpp"

namespace LXNavigation
{
namespace Binary
{
namespace
{
enum class Codes
{
  STX = 0x02,
  ACK = 0x06,
  NACK = 0x15,
};

enum class Commands
{
  SYN = 0x16,
  GET_LOGGER_INFO = 0xC4,
  SET_TASK = 0xCA,
  GET_TASK = 0xCB,
  SET_CLASS = 0xD0,
  GET_FLIGHT_INFO = 0xF0,
  GET_FLIGHT_BLOCK = 0xF1,
  GET_NUMBER_FLIGHTS = 0xF2,
  SEND_TO_RADIO = 0xF3,
  SET_OBS_ZONE = 0xF4,
  GET_OBS_ZONE = 0xF5
};

struct BinaryObsZoneData
{
  uint8_t tp_number; //!< TP number [example: 0=Takeoff, 1=Start, 2 = TP1, 3=TP2, 4=Finish, 5=landing]
  uint8_t direction; //!< direction [0= Symmetric (default), 1=Fixed, 2=Next, 3=Previous, Start]
  uint8_t auto_next; //!< Is this auto next TP or AAT TP
  uint8_t is_line; //!< Is this line flag
  float a1; //!< Angle A1 in radians
  float a2; //!< Angle A2 in radians
  float a21; //!< Angle A21 in radians
  uint32_t r1; //!< Radius R1 in meters
  uint32_t r2; //!< Radius R2 in meters
  float elevation; //!< Turnpoint elevation
}__attribute__ ((packed)); //size 28byte
static_assert(sizeof(BinaryObsZoneData) == 28);

struct BinaryFlightInfo
{
  uint16_t uiFlightID; //< Flight id
  char acIGCFileName[10]; //< IGC file name for file copy
  uint32_t uiDate; //< Date (Julian day)
  uint32_t uiTakeOff; //< Takeoff time (seconds after midnight)
  uint32_t uiLanding; //< Landing time (seconds after midnight)
  char acName[12]; //< Pilot name
  char acSurname[12]; //< Pilot surname
  char acRegNr[8]; //< Registration number.
  char acCompId[8]; //< Competition ID.
  int8_t iMinGforce; //< Minimum G-force (need to be divided by 10)
  int8_t iMaxGforce; //< Maximum G-force (need to be divided by 10)
  uint16_t uiMaxALT; //< Maximum altitude
  uint16_t uiMaxIAS; //< Maximum indicated air speed
  //! Free space for future.
  uint8_t m_abyFree[16];
}__attribute__ ((packed));
static_assert(sizeof(BinaryFlightInfo) == 86);

struct BinaryEosFlightInfo
{
  BinaryFlightInfo m_fi; //< basic flight info struct
  uint32_t m_iSize; //< flight file size
}__attribute__ ((packed));
static_assert(sizeof(BinaryEosFlightInfo) == 90);

struct BinaryOldFlight {
  uint8_t flag; //< Not used
  uint16_t oo_id; //< Not used
  char pilot[19]; //< "Name Surname"
  char glider[12]; //< Polar name
  char reg_num[8]; //< Acft registration number
  char cmp_num[4]; //< Competition id
  uint8_t m_glider_class; //< 0=STANDARD, 1=15-METER, 2=OPEN,
  //< 3=18-METER, 4=WORLD, 5=DOUBLE,
  //< 6=MOTOR_GL
  char observer[10]; //< Not used
  uint8_t gpsdatum; //< Not used
  uint8_t fix_accuracy; //< Not used
  char gps[60]; //< Not used
}__attribute__ ((packed)); //size 119byte
static_assert(sizeof(BinaryOldFlight) == 119);

constexpr int max_tp_number = 12;
struct BinaryTask
{
  /* auto defined */
  uint8_t flag; //< Not used
  int32_t input_time; //< Not used
  uint8_t di; //< Not used
  uint8_t mi; //< Not used
  uint8_t yi; //< Not used
  /* user defined */
  uint8_t fd; //< Not used
  uint8_t fm; //< Not used
  uint8_t fy; //< Not used
  int16_t taskid; //< Not used
  char num_of_tp; //< Number of TP without Takeoff,
  //< Start, Finish and Landing.
  uint8_t prg[max_tp_number]; //< 1=Turnpoint (also Start
  //< and Finish), 2=Landing, 3=Takeoff
  int32_t lon[max_tp_number]; //< TP Longitude in degrees
  //< multiplied by 60000.0f
  int32_t lat[max_tp_number]; //< TP Latitude in degrees
  //< multiplied by 60000.0f
  char name[max_tp_number][9]; //< TP Name
}__attribute__ ((packed)); //size 230byte
static_assert(sizeof(BinaryTask) == 230);

constexpr std::size_t max_binary_packet_size = std::max(sizeof(BinaryOldFlight) + sizeof(BinaryTask), std::max(sizeof(BinaryEosFlightInfo), sizeof(BinaryObsZoneData))) + 1;

uint8_t CalcCRC(const uint8_t* const datagram, uint8_t size)
{
  constexpr uint8_t crc_poly = 0x69;

  uint8_t calculated_crc = 0xff;

  for (uint8_t byte = 0; byte <= size; ++byte)
  {
    int8_t datagram_byte = datagram[byte];
    int8_t tmp = datagram_byte;
    for (uint8_t bit = 0; ++bit <= 8; datagram_byte <<= 1)
    {
      tmp = calculated_crc^datagram_byte;
      calculated_crc <<= 1;
      if (tmp < 0)
        calculated_crc ^= crc_poly;
    }
  }
  return calculated_crc;
}
}

static_assert(max_packet_size >= max_binary_packet_size, "Generic packet size can't fit well-known packets");

void GenerateLoggerInfoGet(PacketBuffer& data)
{
}

std::optional<DeviceInfo> ParseLoggerInfo(const PacketBuffer& data)
{
  CalcCRC(data.data(), data.size());
  return {};
}


void GenerateClass(PacketBuffer& data)
{}


void GenerateTaskGet(PacketBuffer& data)
{}

void GenerateTask(const TurnpointDataContainer& device_info, PacketBuffer& data)
{}

std::optional<TurnpointDataContainer> ParseTask(const PacketBuffer& data)
{
  return {};
}


void GenerateFlightNumberGet(PacketBuffer& data)
{}

std::optional<uint16_t> ParseFlightNumber(const PacketBuffer& data)
{
  return {};
}


void GenerateFlightInfoGet(uint16_t flight_id, PacketBuffer& data)
{}

std::optional<FlightInfo> ParseFlightInfo(const PacketBuffer& data)
{
  return {};
}


void GenerateFlightBlockGet(uint16_t flight_id, uint16_t flight_block, PacketBuffer& data)
{}

void ParseFlightBlock(const FlightBlockBuffer& data, FlightBlockBuffer& block_data)
{}


void GenerateTurnpointZoneGet(PacketBuffer& data)
{}

void GenerateTurnpointZone(const TurnpointZone& zone, PacketBuffer& data)
{}

std::optional<TurnpointZone> ParseTurnpointZone(const PacketBuffer& data)
{
  return {};
}


void GenerateSendToRadio(PacketBuffer& data, const PacketBuffer& radio_data)
{}

bool ParseResult(const PacketBuffer &data)
{
  return true;
}


}
}

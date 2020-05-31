#include "BinaryProtocol.hpp"

namespace LXNavigation
{
namespace Binary
{
namespace
{
#define CRCPOLY 0x69

uint8_t CalcCRC(const uint8_t* const datagram, uint8_t size)
{
  uint8_t m_byCrc = 0xff;

  for (uint8_t byte = 0; byte <= size; ++byte) {
    int8_t d = datagram[byte];
    int8_t tmp = d;
    for (uint8_t bit = 0; ++bit <= 8; d <<= 1) {
        tmp = m_byCrc^d;
        m_byCrc <<= 1;
        if (tmp < 0)
          m_byCrc ^= CRCPOLY;
    }
  }
  return m_byCrc;
}
}

void GetFlightBlock(uint16_t flight_id, uint16_t flight_block, uint8_t* block_data)
{
  CalcCRC(block_data, 10);
}

}
}

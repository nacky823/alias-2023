#ifndef FLASH_HPP_
#define FLASH_HPP_

#include "stm32f4xx_hal.h"

class Flash
{
public:
    bool Clear();
    bool StoreUint8(uint32_t, uint8_t *, uint32_t);
    bool StoreUint16(uint32_t, uint16_t *, uint32_t);
    bool StoreInt16(uint32_t, int16_t *, uint32_t);
    bool StoreFloat(uint32_t, float *, uint32_t);
    void Load(void *, uint32_t, uint32_t);
    bool CheckBlankByte(uint32_t, uint32_t);
    bool CheckBlankHalfword(uint32_t, uint32_t);
    bool CheckBlankWord(uint32_t, uint32_t);

};

/* Edit linker script.
 *
 * MEMORY
 * {
 *   CCMRAM       (xrw) : ORIGIN = 0x10000000,   LENGTH = 64K
 *   RAM          (xrw) : ORIGIN = 0x20000000,   LENGTH = 128K
 *   FLASH_SECTOR0 (rx) : ORIGIN = 0x08000000,   LENGTH = 16K
 *   FLASH_DATA     (r) : ORIGIN = 0x08004000,   LENGTH = 112K
 *   FLASH         (rx) : ORIGIN = 0x08020000,   LENGTH = 896K
 * }
 *
 */

#endif // FLASH_HPP_
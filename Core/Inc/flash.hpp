#ifndef FLASH_HPP_
#define FLASH_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

class Flash
{
public:
    bool Clear();
    bool StoreUint16(uint32_t, uint16_t *, uint32_t);
    bool StoreInt16(uint32_t, int16_t *, uint32_t);
    bool StoreFloat(uint32_t, float *, uint32_t);
    void Load(void *, uint32_t, uint32_t);

};

#endif // FLASH_HPP_
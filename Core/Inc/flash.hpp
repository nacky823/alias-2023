#ifndef FLASH_HPP_
#define FLASH_HPP_

class Flash
{
public:
    bool Clear();
    bool StoreUint16(uint32_t, uint16_t *, uint32_t);
    bool StoreInt16(uint32_t, int16_t *, uint32_t);
    bool StoreFloat(uint32_t, float *, uint32_t);
    void LoadUint16(uint16_t *, uint32_t, uint32_t);
    void LoadInt16(int16_t *, uint32_t, uint32_t);
    void LoadFloat(float *, uint32_t, uint32_t);
}
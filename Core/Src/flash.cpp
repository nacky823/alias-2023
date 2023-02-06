#include "flash.hpp"

const uint32_t sector_1_address_head = 0x08004000; // 16 [KB]
const uint32_t sector_1_address_tail = 0x08007FFF;
const uint32_t sector_2_address_head = 0x08008000; // 16 [KB]
const uint32_t sector_2_address_tail = 0x0800BFFF;
const uint32_t sector_3_address_head = 0x0800C000; // 16 [KB]
const uint32_t sector_3_address_tail = 0x0800FFFF;
const uint32_t sector_4_address_head = 0x08010000; // 64 [KB]
const uint32_t sector_4_address_tail = 0x0801FFFF;


#include "flash.hpp"

#define SECTOR_1_ADDRESS_HEAD 0x08004000 // 16 [KB]
#define SECTOR_1_ADDRESS_TAIL 0x08007FFF
#define SECTOR_2_ADDRESS_HEAD 0x08008000 // 16 [KB]
#define SECTOR_2_ADDRESS_TAIL 0x0800BFFF
#define SECTOR_3_ADDRESS_HEAD 0x0800C000 // 16 [KB]
#define SECTOR_3_ADDRESS_TAIL 0x0800FFFF
#define SECTOR_4_ADDRESS_HEAD 0x08010000 // 64 [KB]
#define SECTOR_4_ADDRESS_TAIL 0x0801FFFF

/*
const uint32_t sector_1_address_head = 0x08004000; // 16 [KB]
const uint32_t sector_1_address_tail = 0x08007FFF;
const uint32_t sector_2_address_head = 0x08008000; // 16 [KB]
const uint32_t sector_2_address_tail = 0x0800BFFF;
const uint32_t sector_3_address_head = 0x0800C000; // 16 [KB]
const uint32_t sector_3_address_tail = 0x0800FFFF;
const uint32_t sector_4_address_head = 0x08010000; // 64 [KB]
const uint32_t sector_4_address_tail = 0x0801FFFF;
*/

bool Flash::Clear()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_SECTOR_1;
    erase.NbSectors    = 4;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    // Success => 0xFFFFFFFF is stored.
    // Fail => Sector number is stored.
    uint32_t failed_sector = 0;

    if(HAL_FLASHEx_Erase(&erase, &failed_sector) != HAL_OK)
    {
#ifdef DEBUG_MODE
        g_erase_failed_sector = failed_sector;
#endif // DEBUG_MODE

        return false;
    }

    HAL_FLASH_Lock();

    return failed_sector == 0xFFFFFFFF;
}

bool Flash::StoreUint16(uint32_t address, uint16_t *data, uint32_t number)
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef result;
    
    for(uint32_t i = 0; i < number; i++)
    {
        result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data++);
        address += 2;
        if(result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}

bool Flash::StoreInt16(uint32_t address, int16_t *data, uint32_t number)
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef result;
    
    for(uint32_t i = 0; i < number; i++)
    {
        result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data++);
        address += 2;
        if(result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}

bool Flash::StoreFloat(uint32_t address, float *data, uint32_t number)
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef result;
    
    for(uint32_t i = 0; i < number; i++)
    {
        result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data++);
        address += 4;
        if(result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}
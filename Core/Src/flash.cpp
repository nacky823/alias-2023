#include "flash.hpp"
#include "declare_extern.h"
#include <string.h>

bool Flash::Clear()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS; // Erasure range : Sectors
    erase.Sector       = FLASH_SECTOR_1;          // Which sector : 1
    erase.NbSectors    = 4;                       // How many sector : 4 (1,2,3,4)
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // Voltage range : 2.7 ~ 3.6 [V]

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

bool Flash::StoreUint8(uint32_t address, uint8_t *data, uint32_t number)
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef result;
    
    for(uint32_t i = 0; i < number; i++)
    {
        result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address++, *data++);
        if(result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
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

void Flash::Load(void *data, uint32_t address, uint32_t size)
{
    memcpy(data, reinterpret_cast<uint32_t*>(address), size);
}

bool Flash::CheckBlankByte(uint32_t address, uint32_t number)
{
    uint8_t byte[number];

    memcpy(byte, reinterpret_cast<uint32_t*>(address), number);

    for(uint32_t i = 0; i < number; i++) if(byte[i] != 0xFF) return false;

    return true;
}

bool Flash::CheckBlankHalfword(uint32_t address, uint32_t number)
{
    uint16_t half[number];

    memcpy(half, reinterpret_cast<uint32_t*>(address), number*2);

    for(uint32_t i = 0; i < number; i++) if(half[i] != 0xFFFF) return false;

    return true;
}

bool Flash::CheckBlankWord(uint32_t address, uint32_t number)
{
    uint32_t word[number];

    memcpy(word, reinterpret_cast<uint32_t*>(address), number*4);

    for(uint32_t i = 0; i < number; i++) if(word[i] != 0xFFFFFFFF) return false;

    return true;
}
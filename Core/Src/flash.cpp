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

    // Fail => Sector number is stored.
    // Success => 0xFFFFFFFF is stored.
    uint32_t failed_sector = 0;
    HAL_StatusTypeDef erase_result = HAL_FLASHEx_Erase(&erase, &failed_sector);

    HAL_FLASH_Lock();

    return erase_result == HAL_OK && failed_sector == 0xFFFFFFFF;
}

if(!Clear()) return false;

bool Flash::Store()
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef write_result;
    uint32_t address = SECTOR_1_ADDRESS_HEAD;
    
    for(int i = 0; i < MAX_LOG_INDEX; i++)
    {
        write_result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data);
        result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data);
    }


}
#ifndef __HARDWARESDIO_H
#define __HARDWARESDIO_H
#include "stm32f4xx_hal.h"
#include "Core.h"


#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#define MSD_OK                   ((uint8_t)0x00)
#define MSD_ERROR                ((uint8_t)0x01)
#define SD_TRANSFER_OK           ((uint8_t)0x00)
#define SD_TRANSFER_BUSY         ((uint8_t)0x01)
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
#define SD_DATATIMEOUT           ((uint32_t)100000000)

class HardwareSDIO
{
public:
    HardwareSDIO() noexcept;

    uint8_t Init(void) noexcept;
    uint8_t ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept;
    uint8_t WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept;
    uint8_t Erase(uint32_t StartAddr, uint32_t EndAddr) noexcept;
    uint8_t GetCardState(void) noexcept;
    void GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo) noexcept;
    uint8_t IsDetected(void) noexcept;
    static HardwareSDIO SDIO1;

private:
    SD_HandleTypeDef hsd;
};

#endif

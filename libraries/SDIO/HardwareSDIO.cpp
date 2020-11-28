
#include "HardwareSDIO.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

HardwareSDIO HardwareSDIO::SDIO1;

extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsdio)
{
  TaskBase::GiveFromISR(HardwareSDIO::SDIO1.waitingTask);
}

extern "C" void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsdio)
{
  TaskBase::GiveFromISR(HardwareSDIO::SDIO1.waitingTask);
}    

extern "C" void DMA2_Stream3_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSDIO::SDIO1.dmaRx));    
}

extern "C" void DMA2_Stream6_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSDIO::SDIO1.dmaTx));    
}

volatile uint32_t abortCalled = false;
extern "C" void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
  abortCalled = true;
}

extern "C" void SDIO_IRQHandler()
{
  HAL_SD_IRQHandler(&(HardwareSDIO::SDIO1.hsd));
}

HardwareSDIO::HardwareSDIO() noexcept
{   
}


void HardwareSDIO::initDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, IRQn_Type irq, uint32_t dir, uint32_t minc) noexcept
{
    hdma.Instance                 = inst;
    
    hdma.Init.Channel             = chan;
    hdma.Init.Direction           = dir;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = minc;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.Mode                = DMA_PFCTRL;
    hdma.Init.Priority            = DMA_PRIORITY_LOW;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst            = DMA_MBURST_INC4;
    hdma.Init.PeriphBurst         = DMA_PBURST_INC4;
    
    if (HAL_DMA_Init(&hdma) != HAL_OK)
    {
      debugPrintf("Failed to init DMA\n");
      delay(5000);
    }
    NVIC_EnableIRQ(irq);      
}

/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t HardwareSDIO::Init(void) noexcept
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if (IsDetected() != SD_PRESENT) {
    return MSD_ERROR;
  }
  __HAL_RCC_SDIO_CLK_ENABLE();
  /* HAL SD initialization */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  pinmap_pinout(PC_8, PinMap_SD);
  pinmap_pinout(PC_9, PinMap_SD);
  pinmap_pinout(PC_10, PinMap_SD);
  pinmap_pinout(PC_11, PinMap_SD);
  pinmap_pinout(PC_12, PinMap_SD);
  pinmap_pinout(PD_2, PinMap_SD);
  // DMA setup
  __HAL_RCC_DMA2_CLK_ENABLE();
  initDmaStream(dmaRx, DMA2_Stream3, DMA_CHANNEL_4, DMA2_Stream3_IRQn, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
  initDmaStream(dmaTx, DMA2_Stream6, DMA_CHANNEL_4, DMA2_Stream6_IRQn, DMA_MEMORY_TO_PERIPH, DMA_MINC_ENABLE);
  __HAL_LINKDMA(&hsd, hdmarx, dmaRx);
  __HAL_LINKDMA(&hsd, hdmatx, dmaTx);
  waitingTask = 0;
  sd_state = HAL_SD_Init(&hsd);
  /* Configure SD Bus width (4 bits mode selected) */
  if (sd_state == MSD_OK) {
#if 0
    /* Enable wide operation */
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
      sd_state = MSD_ERROR;
    }
#endif
  }

  return sd_state;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t HardwareSDIO::ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept
{
  uint8_t sd_state = MSD_OK;
uint32_t start = millis();

while(HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER && millis() - start < 5000)
{

}
if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
{
  debugPrintf("Card not ready\n");
  return MSD_ERROR;
}
if (((uint32_t)pData & 3) != 0)
{
  debugPrintf("Bad address\n");
  delay(5000);
}
//debugPrintf("Start read\n");
  waitingTask = TaskBase::GetCallerTaskHandle();
  HAL_StatusTypeDef stat = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  if (stat != HAL_OK) {
    debugPrintf("Read %d len %d error %d\n", ReadAddr, NumOfBlocks, stat);
    return MSD_ERROR;
  }
  if(!TaskBase::Take(Timeout)) // timed out
  {
      sd_state = MSD_ERROR;
      debugPrintf("Read SD timeout\n");
  }
  //debugPrintf("Read complete\n");
  waitingTask = 0;
  //debugPrintf("abort called %d\n", abortCalled);
  return sd_state;
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t HardwareSDIO::WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout) noexcept
{
  uint8_t sd_state = MSD_OK;
uint32_t start = millis();
while(HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER && millis() - start < 5000)
{

}
if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
{
  debugPrintf("Card not ready\n");
  return MSD_ERROR;
}
  waitingTask = TaskBase::GetCallerTaskHandle();
  HAL_StatusTypeDef stat = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  if (stat != HAL_OK) {
    debugPrintf("Write %d len %d error %d\n", WriteAddr, NumOfBlocks, stat);
    return MSD_ERROR;
  }
  if(!TaskBase::Take(Timeout)) // timed out
  {
      sd_state = MSD_ERROR;
      debugPrintf("Write SD timeout\n");
  }
  waitingTask = 0;

  return sd_state;
}



/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t HardwareSDIO::Erase(uint32_t StartAddr, uint32_t EndAddr) noexcept
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_Erase(&hsd, StartAddr, EndAddr) != HAL_OK) {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t HardwareSDIO::GetCardState(void) noexcept
{
  return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER ) ?
    SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
void HardwareSDIO::GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo) noexcept
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd, CardInfo);
}


/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
uint8_t HardwareSDIO::IsDetected(void) noexcept
{
  __IO uint8_t status = SD_PRESENT;
  return status;
}

#if 0
void HAL_SD_MspInit(SD_HandleTypeDef* hsd) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hsd->Instance==SDIO) {
    /* Peripheral clock enable */
    __HAL_RCC_SDIO_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


}

/**
* @brief SD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd) {
  if(hsd->Instance==SDIO) {
    /* Peripheral clock disable */
    __HAL_RCC_SDIO_CLK_DISABLE();

    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  }
}
#endif


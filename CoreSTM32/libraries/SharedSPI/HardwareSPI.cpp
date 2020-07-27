//Hardware SPI
#include "HardwareSPI.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "spi_com.h"

// Create SPI devices with default pins, these may get changed to alternates later
HardwareSPI HardwareSPI::SSP1(PA_5, PA_6, PB_5, PA_4);
HardwareSPI HardwareSPI::SSP2(PB_13, PB_14, PB_15, PB_12);
HardwareSPI HardwareSPI::SSP3(PC_10, PC_11, PC_12, PA_15);

//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static inline void flushTxFifo(SPI_HandleTypeDef *sspDevice) noexcept
{

}

static inline void flushRxFifo(SPI_HandleTypeDef *sspDevice) noexcept
{
}


// Disable the device and flush any data from the fifos
void HardwareSPI::disable() noexcept
{

}

// Wait for transmitter empty returning true if timed out
//static inline bool waitForTxEmpty(LPC_SSP_TypeDef* sspDevice)
bool HardwareSPI::waitForTxEmpty() noexcept
{
    return false;
}

static inline uint32_t getSSPBits(uint8_t bits) noexcept
{

    return 8;
}


static inline uint32_t getSSPMode(uint8_t spiMode) noexcept
{
    return 0;
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) noexcept
{
    HardwareSPI *s = &HardwareSPI::SSP1;
    if (&(s->spi.handle) == hspi)
        if (s->callback) s->callback(s);    
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) noexcept
{
    HardwareSPI *s = &HardwareSPI::SSP1;
    if (&(s->spi.handle) == hspi)
        if (s->callback) s->callback(s);    
}    

extern "C" void DMA2_Stream2_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaRx));    
}

extern "C" void DMA2_Stream3_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaTx));    
}
// Called on completion of a blocking transfer
void transferComplete(HardwareSPI *spiDevice) noexcept
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(spiDevice->waitingTask, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HardwareSPI::initPins(Pin sck, Pin miso, Pin mosi, Pin cs) noexcept
{
    spi.pin_sclk = sck;
    spi.pin_miso = miso;
    spi.pin_mosi = mosi;
    spi.pin_ssel = csPin = cs;   
}

void HardwareSPI::initDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept
{
    hdma.Instance                 = inst;
    
    hdma.Init.Channel             = chan;
    hdma.Init.Direction           = dir;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = minc;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_LOW;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    
    HAL_DMA_Init(&hdma);   
}
void HardwareSPI::initDma() noexcept
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    // init channels
    initDmaStream(dmaRx, DMA2_Stream2, DMA_CHANNEL_3, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
    initDmaStream(dmaTx, DMA2_Stream3, DMA_CHANNEL_3, DMA_MEMORY_TO_PERIPH, DMA_MINC_ENABLE);
    // link them in
    __HAL_LINKDMA(&(spi.handle), hdmatx, dmaTx);
    __HAL_LINKDMA(&(spi.handle), hdmarx, dmaRx);
    // Enable
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);    
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);    
}

void HardwareSPI::configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept
{
    Pin cs = (hardwareCS ? csPin : NoPin);
    if (!initComplete || bitRate != curBitRate || bits != curBits || clockMode != curClockMode )
    {
        if (!initComplete)
            initDma();
        spi.pin_ssel = cs;
        spi_init(&spi, bitRate, (spi_mode_e)clockMode, 1);
        initComplete = true;
        curBitRate = bitRate;
        curBits = bits;
        curClockMode = clockMode;
    }
}


//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    configureDevice(SPI_MODE_MASTER, bits, clockMode, bitRate, false);
}


HardwareSPI::HardwareSPI(Pin clk, Pin miso, Pin mosi, Pin cs) noexcept :initComplete(false)
{
    spi.pin_sclk = clk;
    spi.pin_miso = miso;
    spi.pin_mosi = mosi;
    spi.pin_ssel = csPin = cs;
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;  
}

void HardwareSPI::startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete) noexcept
{
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&(spi.handle));
    if (state != HAL_SPI_STATE_READY)
    {
        debugPrintf("SPI not ready %x\n", state);
        delay(100);
    }
    HAL_DMA_StateTypeDef dmaState = HAL_DMA_GetState(spi.handle.hdmarx);
    if (dmaState != HAL_DMA_STATE_READY)
    {
        debugPrintf("RX DMA not ready %x\n", dmaState);
        delay(100);
    }
    dmaState = HAL_DMA_GetState(spi.handle.hdmatx);
    if (dmaState != HAL_DMA_STATE_READY)
    {
        debugPrintf("TX DMA not ready %x\n", dmaState);
        delay(100);
    }

    HAL_StatusTypeDef status;    
    callback = ioComplete;
    if (rx_data == nullptr)
        status = HAL_SPI_Transmit_DMA(&(spi.handle), (uint8_t *)tx_data, len);
    else if (tx_data == nullptr)
        status = HAL_SPI_TransmitReceive_DMA(&(spi.handle), rx_data, rx_data, len);
    else
        status = HAL_SPI_TransmitReceive_DMA(&(spi.handle), (uint8_t *)tx_data, rx_data, len);
    if (status != HAL_OK)
        debugPrintf("SPI Error %d\n", (int)status);
}

spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    waitingTask = xTaskGetCurrentTaskHandle();
    startTransfer(tx_data, rx_data, len, transferComplete);
    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
    if( ulTaskNotifyTake(pdTRUE, xDelay) == 0) // timed out
    {
        ret = SPI_TIMEOUT;
        debugPrintf("SPI timeout\n");
    }
    return ret;
}

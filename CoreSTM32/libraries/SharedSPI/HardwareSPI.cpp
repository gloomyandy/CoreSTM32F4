//Hardware SPI
#include "HardwareSPI.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "spi_com.h"

#define SPI_TIMEOUT       15000

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

extern "C"  void SSP0_IRQHandler(void) noexcept
{
    HardwareSPI *s = &HardwareSPI::SSP1;
    if (s->callback) s->callback(s);    
}

extern "C"  void SSP1_IRQHandler(void) noexcept
{
    HardwareSPI *s = &HardwareSPI::SSP1;
    if (s->callback) s->callback(s);
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

void HardwareSPI::configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept
{
    Pin cs = (hardwareCS ? csPin : NoPin);
    if (!initComplete || bitRate != curBitRate || bits != curBits || clockMode != curClockMode )
    {
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

}

spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    spi_status_e stmRet = SPI_OK;
    if (rx_data == nullptr)
        stmRet = spi_send(&spi, (uint8_t *)tx_data, len, SPITimeoutMillis);
    else if (tx_data == nullptr)
        stmRet = spi_transfer(&spi, rx_data, rx_data, len, SPITimeoutMillis);
    else
        stmRet = spi_transfer(&spi,  (uint8_t *)tx_data, rx_data, len, SPITimeoutMillis);

    if (stmRet == SPI_OK)
        return SPI_OK;
    else
        return SPI_ERROR;
#if 0
    waitingTask = xTaskGetCurrentTaskHandle();
    startTransfer(tx_data, rx_data, len, transferComplete);
    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
    if( ulTaskNotifyTake(pdTRUE, xDelay) == 0) // timed out
    {
        ret = SPI_ERROR_TIMEOUT;
    }
    return ret;
#endif
}

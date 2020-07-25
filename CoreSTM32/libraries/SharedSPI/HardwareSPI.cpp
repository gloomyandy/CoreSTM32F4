//Hardware SPI
#include "HardwareSPI.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define SPI_TIMEOUT       15000
// Pin usage, SSP1 has a fixed set of pins SSP0 has two alternatives for each pin
static constexpr Pin SSP1Pins[] = {};
static Pin SSP0Pins[] = {};

HardwareSPI HardwareSPI::SSP0((SPI_HandleTypeDef*)0, SSP0Pins);
HardwareSPI HardwareSPI::SSP1((SPI_HandleTypeDef*)1, (Pin *)SSP1Pins);

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

void HardwareSPI::configurePins(bool hardwareCS) noexcept
{
    // Attach the SSP module to the I/O pins, note that SSP0 can use pins either on port 0 or port 1

}

void HardwareSPI::initPins(Pin sck, Pin miso, Pin mosi, Pin cs) noexcept
{
   
}

void HardwareSPI::configureBaseDevice() noexcept
{

}

void HardwareSPI::configureMode(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{

}

void HardwareSPI::configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept
{

}


//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{

}


HardwareSPI::HardwareSPI(SPI_HandleTypeDef *sspDevice, Pin* spiPins) noexcept :needInit(true), pins(spiPins)
{
    ssp = sspDevice;    
}

void HardwareSPI::startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete) noexcept
{    

}

spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    waitingTask = xTaskGetCurrentTaskHandle();
    startTransfer(tx_data, rx_data, len, transferComplete);
    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
    if( ulTaskNotifyTake(pdTRUE, xDelay) == 0) // timed out
    {
        ret = SPI_ERROR_TIMEOUT;
    }
    return ret;
}

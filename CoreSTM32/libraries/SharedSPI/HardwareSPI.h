#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "Core.h"
#include "SPI.h"

#include "FreeRTOS.h"
#include "task.h"
#include "spi_com.h"
extern "C" void DMA2_Stream2_IRQHandler(void) noexcept;
extern "C" void DMA2_Stream3_IRQHandler(void) noexcept;

class HardwareSPI;
typedef void (*SPICallbackFunction)(HardwareSPI *spiDevice) noexcept;
class HardwareSPI: public SPI
{
public:
    HardwareSPI(Pin clk, Pin miso, Pin mosi, Pin cs) noexcept;
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    bool waitForTxEmpty() noexcept;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept; // Master mode
    void configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, bool hardwareCS) noexcept;
    void initPins(Pin sck, Pin miso, Pin mosi, Pin cs = NoPin) noexcept;
    void disable() noexcept;
    void startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete) noexcept;
    static HardwareSPI SSP1;
    static HardwareSPI SSP2;
    static HardwareSPI SSP3;

private:
    spi_t spi;
    DMA_HandleTypeDef dmaRx;
    DMA_HandleTypeDef dmaTx;
    Pin csPin;
    uint32_t curBitRate;
    uint32_t curBits;
    uint32_t curClockMode;
    bool initComplete;
    SPICallbackFunction callback;
    TaskHandle_t waitingTask;

    void initDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef* inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept;
    void initDma() noexcept;

    friend void DMA2_Stream2_IRQHandler() noexcept;
    friend void DMA2_Stream3_IRQHandler() noexcept;
    friend void transferComplete(HardwareSPI *spiDevice) noexcept;
    friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
    friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
};

#endif

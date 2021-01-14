//Author: sdavi

//SoftwareSPI


#include "SoftwareSPI.h"

// The following three constatnts provide a rough estimate of how many clock cycles are required for one
// bit transfer. These can then be scaled based on mcu speed to adjust the actual SPI timing to match the
// requested transfer rate.

static constexpr uint32_t fastCycleCount = 28;
static constexpr uint32_t slowCycleCount = 50;
static constexpr uint32_t timingCycleCount = 18;

//#define SWSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

SoftwareSPI SoftwareSPI::SWSSP0;

bool SoftwareSPI::waitForTxEmpty() noexcept
{
    return false;
}


void SoftwareSPI::initPins(Pin clk, Pin miso, Pin mosi, Pin cs, DMA_Stream_TypeDef* rxStream, uint32_t rxChan, IRQn_Type rxIrq,
                            DMA_Stream_TypeDef* txStream, uint32_t txChan, IRQn_Type txIrq) noexcept
{
    this->sck = clk;
    this->miso = miso;
    this->mosi = mosi;
}

//setup the master device.
void SoftwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    if(needInit)
    {
        pinMode(miso, INPUT_PULLUP);
        pinMode(mosi, OUTPUT_HIGH);
        pinMode(sck, OUTPUT_LOW);
        mode = clockMode;
        // Work out what delays we need to meet the requested bit rate.
        uint32_t targetCycleNanos = 1000000/(bitRate/1000);
        uint32_t fastCycleNanos = (fastCycleCount*1000000)/(SystemCoreClock/1000);
        uint32_t slowCycleNanos = (slowCycleCount*1000000)/(SystemCoreClock/1000);
        uint32_t timingCycleNanos = (timingCycleCount*1000000)/(SystemCoreClock/1000);
        if (targetCycleNanos <= fastCycleNanos)
            delay = 0;
        else if (targetCycleNanos <= slowCycleNanos)
            delay = 1;
        else
            delay = NanosecondsToCycles(targetCycleNanos - timingCycleNanos)/2;
        needInit = false;
    }
}


SoftwareSPI::SoftwareSPI() noexcept
    :needInit(true),sck(NoPin),mosi(NoPin),miso(NoPin),mode(0)
{

}

spi_status_t SoftwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    for (uint32_t i = 0; i < len; ++i)
    {
        uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
        uint8_t dIn = transfer_byte(dOut);
        if(rx_data != nullptr) *rx_data++ = dIn;
    }
	return SPI_OK;
}

/*
 * Simultaneously transmit and receive a byte on the SPI.
 *
 * Supports mode 0 and mode 1. Does not currently support modes which require an inverted clock
 *
 * Returns the received byte.
 
 //WikiPedia: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Example_of_bit-banging_the_master_protocol
 
 */
uint8_t SoftwareSPI::transfer_byte(uint8_t byte_out) noexcept
{
    uint8_t byte_in = 0;
    uint8_t bit;
    uint32_t start = GetCurrentCycles();
    if (mode & 1)
    {
        if (delay == 0)
        {
            for (bit = 0x80; bit; bit >>= 1) {            
                /* Pull the clock line high */
                digitalWrite(sck, HIGH);
                /* Shift-out a bit to the MOSI line */
                digitalWrite(mosi, (byte_out & bit) ? HIGH : LOW);
                /* Pull the clock line low */
                digitalWrite(sck, LOW);
                /* Shift-in a bit from the MISO line */
                if (digitalRead(miso) == HIGH)
                    byte_in |= bit;
            }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {            
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                digitalWrite(sck, HIGH);
                /* Shift-out a bit to the MOSI line */
                digitalWrite(mosi, (byte_out & bit) ? HIGH : LOW);
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                digitalWrite(sck, LOW);
                /* Shift-in a bit from the MISO line */
                if (digitalRead(miso) == HIGH)
                    byte_in |= bit;
            }
        }
    }
    else
    {
        if (delay == 0)
        {       
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);            
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
            }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);            
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
            }
        }
    }
    return byte_in;
}

/*
 * AnalogInput.cpp
 *
 *  Created on: 2 Apr 2016
 *      Author: David
 */

/*
 * A note on the ADCs in the SAME70
 * The ADCs on the SAME70 suffer from noise as mentioned in the errata for the chip.
 * The ADCs support hardware averaging, so we use that to reduce the noise. However, out tests show that hardware averaging above x16 gives the wrong results
 * if a sequence of channels is converted, except for the last channel converted.
 * Duet 3 uses AFEC1 for thermistor, Vref and Vssa monitoring and for the IO_8_IN pin analog functionality. The voltage monitoring, MCU temperature and all other IO_x_IN pins use AFEC1.
 * So we program the AFECs as follows:
 * - AFEC0 is programmed in x16 averaging mode with all active channels converted in sequence.
 *   The maximum number of clock cycles needed is 12 channels * 16 samples * 23 clock/cycle = 4416. So a 10MHz clock is more than enough to convert the sequence within 1ms.
 * - AFEC1 is programmed in x256 averaging mode and is only asked to convert 1 input per tick.
 *   The maximum number of clock cycles needed is 256 samples * 23 clocks/cycle = 5888. So again a 10MHz clock is sufficient.
 * When averaging mode is used, the current data register is not always the last converted result for the corresponding channel. So we need to save all the values
 * before starting another conversion. Call the AnalogInFinaliseConversion function to do this.
 * In order to make AdcBits a constant, we shift the 16-bit results from ADC1 right 2 bits before returning them.
 */
#include "Core.h"
#include "AnalogIn.h"

// Module initialisation
void AnalogInInit()
{

}

// Enable or disable a channel. Use AnalogCheckReady to make sure the ADC is ready before calling this.
void AnalogInEnableChannel(AnalogChannelNumber channel, bool enable)
{

}

// Read the most recent 12-bit result from a channel
uint16_t AnalogInReadChannel(AnalogChannelNumber channel)
{
	return analogRead(channel);
}


// Start converting the enabled channels
void AnalogInStartConversion(uint32_t channels)
{

}




// Finalise a conversion
void AnalogInFinaliseConversion()
{

}


// Check whether all conversions have been completed since the last call to AnalogStartConversion
bool AnalogInCheckReady(uint32_t channels)
{
	return true;
}

// Convert an Arduino Due analog pin number to the corresponding ADC channel number
AnalogChannelNumber PinToAdcChannel(uint32_t pin)
{
	return (AnalogChannelNumber)pin;
}

// Get the temperature measurement channel
AnalogChannelNumber GetTemperatureAdcChannel()
{
	return (AnalogChannelNumber)PADC_TEMP;
}
// End

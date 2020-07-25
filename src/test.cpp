//#include "Core.h"
#ifndef Core_h
#define Core_h
#include "ecv.h"		// macros for Escher C/C++ Verifier design-by-contract annotations
#undef array
#undef yield			// eCv definition clashes with function 'yield' in wiring.c (can use _ecv_yield instead within annotations)
#undef value			// needed because we include <optional> in some projects
#undef result

//#include "compiler.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "binary.h"
#include "itoa.h"
#include "PinNames.h"
#include "wiring_constants.h"
//#include "interrupt_stm32.h"
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

//#include "asf.h"

void yield(void) noexcept;
void CoreSysTick(void) noexcept;

typedef PinName Pin;
static const Pin NoPin = NC;
typedef uint8_t DmaChannel;
typedef uint16_t PwmFrequency;		// type used to represent a PWM frequency. 0 sometimes means "default".
typedef uint32_t NvicPriority;

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus

// SSP/SPI Channels
enum SSPChannel : uint8_t
{
    //Hardware SPI
    SSP0 = 0,
    SSP1,
    SSP2,
    //Software SPI
    SWSPI0,

    // Not defined
    SSPNONE = 0xff
};
// Definitions for PWM channels
enum EPWMChannel : int8_t
{
  NOT_ON_PWM=-1,
  PWM_CH0=0,
  PWM_CH1,
  PWM_CH2,
  PWM_CH3,
  PWM_CH4,
  PWM_CH5,
  PWM_CH6,
  PWM_CH7
};

// Definitions for TC channels
enum ETCChannel : int8_t
{
  NOT_ON_TIMER=-1,
  TC0_CHA0=0,
  TC0_CHB0,
  TC0_CHA1,
  TC0_CHB1,
  TC0_CHA2,
  TC0_CHB2,
  TC1_CHA3,
  TC1_CHB3,
  TC1_CHA4,
  TC1_CHB4,
  TC1_CHA5,
  TC1_CHB5,
#if SAM3XA || SAM4E || SAME70
  TC2_CHA6,
  TC2_CHB6,
  TC2_CHA7,
  TC2_CHB7,
  TC2_CHA8,
  TC2_CHB8,
#endif
#if SAME70
  TC3_CHA9,
  TC3_CHB9,
  TC3_CHA10,
  TC3_CHB10,
  TC3_CHA11,
  TC3_CHB11
#endif
};

// The analog input module uses the scheduler in the SAM processor to convert a number of channels.
// Usage:
// 1. Enable the channels you need by making calls to AnalogEnableChannel.
// 2. If desired, call AnalogSetCallback to set a callback for when conversion is complete.
// 3. Call AnalogStartConversion. This may be done e.g. in a tick interrupt if regular conversions are wanted.
// 4. Either use the callback to determine when conversion is complete, or call AnalogCheckReady to poll the status.
// 5. Call AnalogReadChannel to read the most recent converted result for each channel of interest.

enum AnalogChannelNumber : int8_t
{
  NO_ADC=-1,
  ADCC0=0,
  ADCC1,
  ADCC2,
  ADCC3,
  ADCC4,
  ADCC5,
  ADCC6,
  ADCC7,
  ADCC8,
  ADCC9,
  ADCC10,
  ADCC11,
  ADCC12,
  ADCC13,
  ADCC14,
  ADCC15,
  ADCC16,
  ADCC17,
  ADCC18,
  ADCC19,
  ADCC20,
  ADCC21,
  ADCC22,
  ADCC23,
  ADCC24,
  DA0,
  DA1
};

// Pin Attributes to be OR-ed
constexpr uint8_t PIN_ATTR_NONE = 0;
constexpr uint8_t PIN_ATTR_COMBO = 1 << 0;
constexpr uint8_t PIN_ATTR_ANALOG = 1 << 1;
constexpr uint8_t PIN_ATTR_DIGITAL = 1 << 2;
constexpr uint8_t PIN_ATTR_PWM = 1 << 3;
constexpr uint8_t PIN_ATTR_TIMER = 1 << 4;
constexpr uint8_t PIN_ATTR_DAC = 1 << 5;

// Types used for the tables below
typedef GPIO_TypeDef Pio;
struct PinDescription
{
	Pio* pPort;
	uint32_t ulPin;
	uint32_t ulPinConfiguration;
	uint8_t ulPinAttribute;
	AnalogChannelNumber ulADCChannelNumber; // ADC or DAC channel number in the SAM device
	EPWMChannel ulPWMChannel;
	ETCChannel ulTCChannel;
};

/* Pins table to be instantiated into variant.cpp */
extern const PinDescription g_APinDescription[];

//#include "WCharacter.h"
//#include "HardwareSerial.h"
//#include "WInterrupts.h"

#endif // __cplusplus

// Include board variant
//#include "variant.h"

//#include "wiring.h"
//#include "wiring_digital.h"
//#include "watchdog.h"
#include "WMath.h"
//#include "Reset.h"

#ifdef __cplusplus
//#include "AnalogIn.h"
//#include "AnalogOut.h"
//#include "USBSerial.h"
#endif

#endif
#if 0
	template<class X> inline constexpr X min(X _a, X _b) noexcept
	{
		return (_a < _b) ? _a : _b;
	}
#endif
void test(void)
{
    const int a=1;
    const int b=2;
    constexpr int c = min<int>(a, b);
}
/*
 * AnalogInput.cpp
 *
 *  Created 30/7/2020
 *      Author: Andy
 */

/*
Read ADC values. We configure things so that the selected channels are constantly being converted.
Only those channels attached to ADC3 plus a single specual case of the MCU temperature device on
ADC1 are currently supported.
*/
#include "Core.h"
#include "AnalogIn.h"
#include "stm32f4xx_hal_rcc.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

constexpr AnalogChannelNumber ADC_1 = 0x10000;
constexpr AnalogChannelNumber ADC_2 = 0x20000;
constexpr AnalogChannelNumber ADC_3 = 0x30000;

constexpr uint32_t NumChannels = 16; 	// 16 standard chans
static int32_t ChanMap[NumChannels];
static uint32_t ChanValues[NumChannels];
static bool AdcRunning = false;

// Configuration structures for the ADC and Dma
static ADC_HandleTypeDef Adc1Handle = {};
static ADC_HandleTypeDef Adc3Handle = {};
static DMA_HandleTypeDef DmaHandle = {};

/* Private Functions */
static AnalogChannelNumber GetAdcChannel(PinName pin)
{
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    if (function == static_cast<uint32_t>(NC)) return NO_ADC;
    // we return the ADC number in the high 16 bits and the channel number in the low
    ADC_TypeDef* adc = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);
#if 0 
    // We only support ADC3 at the moment
    if (adc == ADC1)
        return (ADC_1 | STM_PIN_CHANNEL(function));
    else if (adc == ADC2)
        return (ADC_2 | STM_PIN_CHANNEL(function));
    else
#endif
    if (adc == ADC3)
        return (ADC_3 | STM_PIN_CHANNEL(function));
    else
    {
        debugPrintf("Unknown ADC device for pin %d\n", static_cast<int>(pin));
        return NO_ADC;
    }
}


static void ConfigureDma()
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    DmaHandle.Instance = DMA2_Stream0;

    DmaHandle.Init.Channel  = DMA_CHANNEL_2;
    DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    DmaHandle.Init.Mode = DMA_CIRCULAR;
    DmaHandle.Init.Priority = DMA_PRIORITY_LOW;
    DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
    DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE; 

    HAL_DMA_Init(&DmaHandle);
    //NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  if (hadc->Instance == ADC1) {
#ifdef __HAL_RCC_ADC1_CLK_ENABLE
    __HAL_RCC_ADC1_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#ifdef ADC2
  else if (hadc->Instance == ADC2) {
#ifdef __HAL_RCC_ADC2_CLK_ENABLE
    __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC3
  else if (hadc->Instance == ADC3) {
#ifdef __HAL_RCC_ADC3_CLK_ENABLE
    __HAL_RCC_ADC3_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC34_CLK_ENABLE
    __HAL_RCC_ADC34_CLK_ENABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC4
  else if (hadc->Instance == ADC4) {
#ifdef __HAL_RCC_ADC34_CLK_ENABLE
    __HAL_RCC_ADC34_CLK_ENABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  }
#endif
#ifdef ADC5
  else if (hadc->Instance == ADC5) {
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_CLK_ENABLE();
#endif
  }
#endif
#ifdef __HAL_RCC_ADC_CLK_ENABLE
  __HAL_RCC_ADC_CLK_ENABLE();
#endif
  /* For STM32F1xx and STM32H7xx, ADC prescaler is configured in
     SystemClock_Config (variant.cpp) */
#if defined(__HAL_RCC_ADC_CONFIG) && !defined(STM32F1xx) && !defined(STM32H7xx)
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
#endif
}

/**
  * @brief  DeInitializes the ADC MSP.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
#ifdef __HAL_RCC_ADC_FORCE_RESET
  __HAL_RCC_ADC_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC_RELEASE_RESET
  __HAL_RCC_ADC_RELEASE_RESET();
#endif

  if (hadc->Instance == ADC1) {
#ifdef __HAL_RCC_ADC1_FORCE_RESET
    __HAL_RCC_ADC1_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC1_RELEASE_RESET
    __HAL_RCC_ADC1_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC12_FORCE_RESET
    __HAL_RCC_ADC12_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC12_RELEASE_RESET
    __HAL_RCC_ADC12_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC1_CLK_DISABLE
    __HAL_RCC_ADC1_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_ADC12_CLK_DISABLE
    __HAL_RCC_ADC12_CLK_DISABLE();
#endif
  }
#ifdef ADC2
  else if (hadc->Instance == ADC2) {
#ifdef __HAL_RCC_ADC2_FORCE_RESET
    __HAL_RCC_ADC2_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC2_RELEASE_RESET
    __HAL_RCC_ADC2_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC12_FORCE_RESET
    __HAL_RCC_ADC12_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC12_RELEASE_RESET
    __HAL_RCC_ADC12_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC2_CLK_DISABLE
    __HAL_RCC_ADC2_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_ADC2_CLK_DISABLE
    __HAL_RCC_ADC2_CLK_DISABLE();
#endif
  }
#endif
#ifdef ADC3
  else if (hadc->Instance == ADC3) {
#ifdef __HAL_RCC_ADC3_FORCE_RESET
    __HAL_RCC_ADC3_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC3_RELEASE_RESET
    __HAL_RCC_ADC3_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC34_FORCE_RESET
    __HAL_RCC_ADC34_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC34_RELEASE_RESET
    __HAL_RCC_ADC34_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC3_CLK_DISABLE
    __HAL_RCC_ADC3_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_ADC34_CLK_DISABLE
    __HAL_RCC_ADC34_CLK_DISABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_FORCE_RESET();
    __HAL_RCC_ADC345_RELEASE_RESET();
    __HAL_RCC_ADC345_CLK_DISABLE();
#endif
  }
#endif
#ifdef ADC4
  else if (hadc->Instance == ADC4) {
#ifdef __HAL_RCC_ADC34_FORCE_RESET
    __HAL_RCC_ADC34_FORCE_RESET();
#endif
#ifdef __HAL_RCC_ADC34_RELEASE_RESET
    __HAL_RCC_ADC34_RELEASE_RESET();
#endif
#ifdef __HAL_RCC_ADC34_CLK_DISABLE
    __HAL_RCC_ADC34_CLK_DISABLE();
#endif
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_FORCE_RESET();
    __HAL_RCC_ADC345_RELEASE_RESET();
    __HAL_RCC_ADC345_CLK_DISABLE();
#endif
  }
#endif
#ifdef ADC5
  else if (hadc->Instance == ADC5) {
#if defined(ADC345_COMMON)
    __HAL_RCC_ADC345_FORCE_RESET();
    __HAL_RCC_ADC345_RELEASE_RESET();
    __HAL_RCC_ADC345_CLK_DISABLE();
#endif
  }
#endif
#ifdef __HAL_RCC_ADC_CLK_DISABLE
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_ADC_CLK_DISABLE();
#endif
}

static void ConfigureAdc1()
{
    // ADC1 is configured to continuously convert a single channel, for mcu temperature 
    // readings
    __HAL_RCC_ADC1_CLK_ENABLE();
    Adc1Handle.Instance = ADC1;
    Adc1Handle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* (A)synchronous clock mode, input ADC clock divided */
    Adc1Handle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
    Adc1Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    Adc1Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    Adc1Handle.Init.EOCSelection          = ADC_EOC_SEQ_CONV ;             /* EOC flag picked-up to indicate conversion end */
    //Adc1Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV ;             /* EOC flag picked-up to indicate conversion end */
 ;             /* EOC flag picked-up to indicate conversion end */
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(STM32F373xC) && !defined(STM32F378xx)
    Adc1Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F3xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G4xx) && \
    !defined(STM32H7xx) && !defined(STM32L4xx) && !defined(STM32WBxx)
    Adc1Handle.Init.LowPowerAutoPowerOff  = DISABLE;                       /* ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered */
#endif
#ifdef ADC_CHANNELS_BANK_A
    Adc1Handle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
#endif
    Adc1Handle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#if !defined(STM32F0xx) && !defined(STM32L0xx)
    Adc1Handle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
#endif
    Adc1Handle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
    Adc1Handle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
#endif
    Adc1Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#if !defined(STM32F1xx)
    Adc1Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
#endif
#if !defined(STM32F1xx) && !defined(STM32H7xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
    Adc1Handle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
#endif
#ifdef ADC_CONVERSIONDATA_DR
    Adc1Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;      /* Regular Conversion data stored in DR register only */
#endif
#ifdef ADC_OVR_DATA_OVERWRITTEN
    Adc1Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
#endif
#ifdef ADC_LEFTBITSHIFT_NONE
    Adc1Handle.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;         /* No bit shift left applied on the final ADC convesion data */
#endif

#if defined(STM32F0xx)
    Adc1Handle.Init.SamplingTimeCommon    = samplingTime;
#endif
#if defined(STM32G0xx)
    Adc1Handle.Init.SamplingTimeCommon1   = samplingTime;              /* Set sampling time common to a group of channels. */
    Adc1Handle.Init.SamplingTimeCommon2   = samplingTime;              /* Set sampling time common to a group of channels, second common setting possible.*/
    Adc1Handle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif
#if defined(STM32L0xx)
    Adc1Handle.Init.LowPowerFrequencyMode = DISABLE;                       /* To be enabled only if ADC clock < 2.8 MHz */
    Adc1Handle.Init.SamplingTime          = samplingTime;
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F3xx) && !defined(STM32F4xx) && !defined(STM32F7xx) && \
    !defined(STM32L1xx)
    Adc1Handle.Init.OversamplingMode      = DISABLE;
  /* AdcHandle.Init.Oversample ignore for STM32L0xx as oversampling disabled */
  /* AdcHandle.Init.Oversampling ignored for other as oversampling disabled */
#endif
#if defined(ADC_CFGR_DFSDMCFG) && defined(DFSDM1_Channel0)
    Adc1Handle.Init.DFSDMConfig           = ADC_DFSDM_MODE_DISABLE;        /* ADC conversions are not transferred by DFSDM. */
#endif
#ifdef ADC_TRIGGER_FREQ_HIGH
    Adc1Handle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif

    Adc1Handle.State = HAL_ADC_STATE_RESET;
    Adc1Handle.DMA_Handle = NULL;
    Adc1Handle.Lock = HAL_UNLOCKED;
    /* Some other ADC_HandleTypeDef fields exists but not required */
    if (HAL_ADC_Init(&Adc1Handle) != HAL_OK) 
    {
        debugPrintf("ADC Init failed\n");
        return;
    }
    // add in the single Temeperature sensor channel
    ADC_ChannelConfTypeDef  AdcChannelConf = {};
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    AdcChannelConf.Offset = 0;
    AdcChannelConf.Rank = 1;
    AdcChannelConf.Channel = ADC_CHANNEL_TEMPSENSOR;
    HAL_ADC_ConfigChannel(&Adc1Handle, &AdcChannelConf);
    // and start it running
    HAL_ADC_Start(&Adc1Handle);
}


static void ConfigureAdc3(uint32_t chanCount)
{
    // Adc3 converts up to 16 channels (but on the Stm32F4 I think we only have 8), continuously and
    // captured to RAM via DMA
    __HAL_RCC_ADC3_CLK_ENABLE();
    Adc3Handle.Instance = ADC3;
    Adc3Handle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* (A)synchronous clock mode, input ADC clock divided */
    Adc3Handle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
    Adc3Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    Adc3Handle.Init.ScanConvMode          = ENABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    Adc3Handle.Init.EOCSelection          = DISABLE;                       /* EOC flag picked-up to indicate conversion end */
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(STM32F373xC) && !defined(STM32F378xx)
    Adc3Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F3xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G4xx) && \
    !defined(STM32H7xx) && !defined(STM32L4xx) && !defined(STM32WBxx)
    Adc3Handle.Init.LowPowerAutoPowerOff  = DISABLE;                       /* ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered */
#endif
#ifdef ADC_CHANNELS_BANK_A
    Adc3Handle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
#endif
    Adc3Handle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#if !defined(STM32F0xx) && !defined(STM32L0xx)
    Adc3Handle.Init.NbrOfConversion       = chanCount;                     /* Specifies the number of ranks that will be converted within the regular group sequencer. */
#endif
    Adc3Handle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
    Adc3Handle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
#endif
    Adc3Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    Adc3Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;

    //Adc3Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#if !defined(STM32F1xx)
    //Adc3Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
#endif
#if !defined(STM32F1xx) && !defined(STM32H7xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
    Adc3Handle.Init.DMAContinuousRequests = ENABLE;                        /* DMA one-shot mode selected (not applied to this example) */
#endif
#ifdef ADC_CONVERSIONDATA_DR
    Adc3Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;      /* Regular Conversion data stored in DR register only */
#endif
#ifdef ADC_OVR_DATA_OVERWRITTEN
    Adc3Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
#endif
#ifdef ADC_LEFTBITSHIFT_NONE
    Adc3Handle.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;         /* No bit shift left applied on the final ADC convesion data */
#endif

#if defined(STM32F0xx)
    Adc3Handle.Init.SamplingTimeCommon    = samplingTime;
#endif
#if defined(STM32G0xx)
    Adc3Handle.Init.SamplingTimeCommon1   = samplingTime;              /* Set sampling time common to a group of channels. */
    Adc3Handle.Init.SamplingTimeCommon2   = samplingTime;              /* Set sampling time common to a group of channels, second common setting possible.*/
    Adc3Handle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif
#if defined(STM32L0xx)
    Adc3Handle.Init.LowPowerFrequencyMode = DISABLE;                       /* To be enabled only if ADC clock < 2.8 MHz */
    Adc3Handle.Init.SamplingTime          = samplingTime;
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F3xx) && !defined(STM32F4xx) && !defined(STM32F7xx) && \
    !defined(STM32L1xx)
    Adc3Handle.Init.OversamplingMode      = DISABLE;
  /* Adc3Handle.Init.Oversample ignore for STM32L0xx as oversampling disabled */
  /* Adc3Handle.Init.Oversampling ignored for other as oversampling disabled */
#endif
#if defined(ADC_CFGR_DFSDMCFG) && defined(DFSDM1_Channel0)
    Adc3Handle.Init.DFSDMConfig           = ADC_DFSDM_MODE_DISABLE;        /* ADC conversions are not transferred by DFSDM. */
#endif
#ifdef ADC_TRIGGER_FREQ_HIGH
    Adc3Handle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif

    Adc3Handle.State = HAL_ADC_STATE_RESET;
    Adc3Handle.Lock = HAL_UNLOCKED;
    /* Some other ADC_HandleTypeDef fields exists but not required */
    if (HAL_ADC_Init(&Adc3Handle) != HAL_OK) 
    {
        debugPrintf("ADC Init failed\n");
        return;
    }
}


static void ConfigureChannels()
{
    ADC_ChannelConfTypeDef  AdcChannelConf = {};

    // Reset everything
    if (AdcRunning)
    {
        HAL_ADC_Stop_DMA(&Adc3Handle);
        // Unfortunately we can't just restart the DMA as it seems to restart at same channel
        // it stopped at which causes a skew in the memory contents. I'm not sure what will 
        // reset the ADC to match the DMA transfer, but call Deinit does the job. Unfortuately
        // it also resets ADC1, so we need to restart that as well!
        HAL_ADC_DeInit(&Adc3Handle);
        ConfigureAdc1();
    }

    uint32_t sampleOffset = 0;
    // first count the active channels
     for(uint32_t i = 0; i < NumChannels; i++)
    {
        if (ChanMap[i] != -1)
            sampleOffset++;
    }
    // setup the ADC for that number of channels
    ConfigureAdc3(sampleOffset);
    // Now add the channels and setup the channel map
    // We do not need to acquire samples particularly quickly, so use use slowest sample time
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    AdcChannelConf.Offset = 0;

    sampleOffset = 0;
    for(uint32_t i = 0; i < NumChannels; i++)
    {
        if (ChanMap[i] != -1)
        {
            // include this channel
            AdcChannelConf.Channel = i;
            AdcChannelConf.Rank = sampleOffset+1;
            HAL_ADC_ConfigChannel(&Adc3Handle, &AdcChannelConf);
            // record the location of the sample
            ChanMap[i] = sampleOffset++;
        }
    }
    // All done restart sampling
    HAL_ADC_Start_DMA(&Adc3Handle, ChanValues, sampleOffset);
    AdcRunning = true;
}

// This is not actually called as we leave the interrupt disabled
extern "C" void DMA2_Stream0_IRQHandler()
{
    HAL_DMA_IRQHandler(&DmaHandle);    
}


// Module initialisation
void AnalogInInit()
{
    // Initially no channels are mapped
    for(uint32_t i = 0; i < NumChannels; i++)
        ChanMap[i] = -1;

    ConfigureDma();
    __HAL_LINKDMA(&Adc3Handle, DMA_Handle, DmaHandle);

    ConfigureAdc1();
}

// Enable or disable a channel. Use AnalogCheckReady to make sure the ADC is ready before calling this.
void AnalogInEnableChannel(AnalogChannelNumber channel, bool enable)
{
    if (channel == (ADC_1 | ADC_CHANNEL_TEMPSENSOR))
        return;
    if ((channel & 0xffff0000) != ADC_3)
    {
        debugPrintf("Unsupported ADC channel %x\n", static_cast<int>(channel));
        return;
    }
    channel &= 0xffff;
    if (channel >= 0 && channel < NumChannels)
    {
        ChanMap[channel] = (enable ? 0 : -1);
        ConfigureChannels();
    }
    else
        debugPrintf("Bad ADC channel %d\n", static_cast<int>(channel));
}

// Read the most recent 12-bit result from a channel
uint16_t AnalogInReadChannel(AnalogChannelNumber channel)
{
    if (channel == (ADC_1 | ADC_CHANNEL_TEMPSENSOR))
    {
        return HAL_ADC_GetValue(&Adc1Handle);
    }
    channel &= 0xffff;
    if (channel >= 0 && channel < NumChannels && ChanMap[channel] >= 0)
    {
        return ChanValues[ChanMap[channel]];
    }
    else
        debugPrintf("ADC channel not configured %d\n", static_cast<int>(channel));
    return 0;
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
    return GetAdcChannel(static_cast<PinName>(pin));
}

// Get the temperature measurement channel
AnalogChannelNumber GetTemperatureAdcChannel()
{
    return (0x10000 | ADC_CHANNEL_TEMPSENSOR);
}
// End

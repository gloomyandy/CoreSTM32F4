#include <Arduino.h>
#include <USBSerial.h>
#include <wiring_time.h>
#include <FreeRTOS.h>
#include <task.h>
#include "RTOSIface/RTOSIface.h"

void delay(uint32_t ms)
{
    vTaskDelay(ms);
}

// Idle task data
constexpr unsigned int IdleTaskStackWords = 40;				// currently we don't use the idle talk for anything, so this can be quite small
static Task<IdleTaskStackWords> idleTask;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) noexcept
{
	*ppxIdleTaskTCBBuffer = idleTask.GetTaskMemory();
	*ppxIdleTaskStackBuffer = idleTask.GetStackBase();
	*pulIdleTaskStackSize = idleTask.GetStackSize();
}



constexpr unsigned int MainTaskStackWords = 1110;			// on other processors we use matrixes of floats


static Task<MainTaskStackWords> mainTask;
extern "C" [[noreturn]] void MainTask(void * pvParameters) noexcept;

static Task<MainTaskStackWords> ledTask;
extern "C" [[noreturn]] void LEDTask(void * pvParameters) noexcept;

#if defined(TARGET_STM32F4)

#endif

extern "C" int _write(int file, char *ptr, int len)
{
    SerialUSB.write(ptr, len);
    return len;
}


void setup()
{
    //SerialUSB.begin(250000);
    //delay(5000);
    //SerialUSB.print("Running....\n");
	idleTask.AddToList();			// add the FreeRTOS internal tasks to the task list
	mainTask.Create(MainTask, "MAIN", nullptr, 2);
	ledTask.Create(LEDTask, "LED", nullptr, 1);
	vTaskStartScheduler();			// doesn't return
    printf("Running2....\n");
}

extern "C" [[noreturn]] void MainTask(void *pvParameters) noexcept
{
    SerialUSB.begin(250000);
    for(;;)
    {
        delay(1000);
        printf("Main thread running\n");
    }
}


uint32_t discoLights[] = {PB_10, PE_12, PG_8, PE_15, PE_10, PG_5};
extern "C" [[noreturn]] void LEDTask(void *pvParameters) noexcept
{
    delay(2000);
    printf("LED task running....");
    pinMode(PA_7, OUTPUT);
    int i = 0;
    for(;;)
    {
        pinMode(discoLights[i], OUTPUT);
        digitalWrite(PA_7, 0);
        delay(500);
        digitalWrite(PA_7, 1);
        delay(500);
        pinMode(discoLights[i], INPUT);
        i = (i+1) % 6;
    }

}


void loop()
{
    delay(1000);
    SerialUSB.print("Looping...\n");
    printf("will this work\n");
    printf("and it still works\n");
}

extern "C" void vAssertCalled( uint32_t ulLine, const char *pcFile ) noexcept
{
    printf("Assert called line %d file %s\n", ulLine, pcFile);
    for(;;) {}
}

extern "C" {
	// This intercepts the 1ms system tick
    void vApplicationTickHook() noexcept
	{
        //HAL_IncTick();
		//CoreSysTick();
		//reprap.Tick();
	}

	[[noreturn]] void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept __attribute((naked));
	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept
	{
    }
}


#ifndef INTERRUPT_LPC_H
#define INTERRUPT_LPC_H

#include <stdbool.h>
#include <stdint.h>
//#include "chip.h"



#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t irqflags_t;
#define cpu_irq_is_enabled()    (__get_PRIMASK() == 0)


#define cpu_irq_enable()                       \
    do {                                       \
        __DMB();                               \
		__enable_irq();                        \
	} while (0)
# define cpu_irq_disable()                     \
	do {                                       \
		__disable_irq();                       \
		__DMB();                               \
	} while (0)


static inline irqflags_t cpu_irq_save(void) noexcept
{
	irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags) noexcept
{
	return (flags);
}

static inline void cpu_irq_restore(irqflags_t flags) noexcept
{
	if (cpu_irq_is_enabled_flags(flags))
		cpu_irq_enable();
}
    
// Return true if we are in any interrupt service routine
static inline bool inInterrupt() noexcept
{
    //bits 0:8 are the ISR_NUMBER
    //bits 9:31 reserved
    return (__get_IPSR() & 0xFF) != 0;
}
    
#ifdef __cplusplus
}
#endif

#endif /* INTERRUPT_LPC_H */

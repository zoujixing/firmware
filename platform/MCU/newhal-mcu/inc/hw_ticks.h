
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#define SYSTEM_US_TICKS		(SystemCoreClock / 1000000)//cycles per microsecond

/**
 * Increment the millisecond tick counter.
 */
void System1MsTick(void);

/**
 * Increment the microsecond tick counter.
 */
void System1UsTick(void);

/**
 * Fetch the current milliseconds count.
 * @return the number of milliseconds since the device was powered on or woken up from
 * sleep. Automatically wraps around when above UINT_MAX;
 */
system_tick_t GetSystem1MsTick();

/**
 * Fetch the current microseconds count.
 * @return the number of microseconds since the device was powered on or woken up from
 * sleep. Automatically wraps around when above UINT_MAX;
 */
system_tick_t GetSystem1UsTick();


#ifdef __cplusplus
}
#endif

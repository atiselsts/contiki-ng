#include "contiki.h"
#include "dev/watchdog.h"

#include "nn.h"

#include <stdio.h> /* For printf() */

typedef void (*DebugLogCallback)(const char* s);

// Registers and application-specific callback for debug logging. It must be
// called before the first call to DebugLog().
void RegisterDebugLogCallback(DebugLogCallback callback);

#define NUM_TESTS 10

void callback(const char* s)
{
  puts(s);
}

void classify(void)
{
  rtimer_clock_t start = RTIMER_NOW();
  int i;
  int dummy = 0;
  for (i = 0; i < NUM_TESTS; ++i) {
    dummy += nn_classify();
    watchdog_periodic();
    //printf("test %d done\n", i);
  }
  rtimer_clock_t end = RTIMER_NOW();
  printf("for %u tests: %lu ticks (%d)\n", NUM_TESTS, (uint32_t)(end - start), dummy);
}

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "Node process");
AUTOSTART_PROCESSES(&node_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  PROCESS_BEGIN();

  RegisterDebugLogCallback(callback);

  if(nn_setup() >= 0) {
    classify();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

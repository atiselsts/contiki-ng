#include "contiki.h"
#include "dev/watchdog.h"

#include "nn.h"

#include <stdio.h> /* For printf() */
#include <string.h>

//typedef void (*DebugLogCallback)(const char* s);

// Registers and application-specific callback for debug logging. It must be
// called before the first call to DebugLog().
//void RegisterDebugLogCallback(DebugLogCallback callback);

#define NUM_TESTS 1

// void callback(const char* s)
// {
//   puts(s);
// }

// override the default debug log in the tensorflow lite micro library
void DebugLog(const char* s) {
  if (s && strlen(s) > 2) {
    printf("contiki: %s", s);
  }
}

void classify(void)
{
  printf("classify\n");

  rtimer_clock_t start = RTIMER_NOW();
  int i;
  int dummy = 0;
  for (i = 0; i < NUM_TESTS; ++i) {
    dummy += nn_classify();
    watchdog_periodic();
    //printf("test %d done\n", i);
  }
  rtimer_clock_t end = RTIMER_NOW();
  printf("for %u tests: %lu ticks, %lu ticks per second (%d)\n",
      NUM_TESTS, (uint32_t)(end - start), (uint32_t)RTIMER_ARCH_SECOND, dummy);
}

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "Node process");
AUTOSTART_PROCESSES(&node_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  PROCESS_BEGIN();

//  RegisterDebugLogCallback(callback);

//  printf("setup\n");

  if(nn_setup() >= 0) {
    classify();
  }

//  printf("end\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

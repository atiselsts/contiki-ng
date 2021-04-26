#include "contiki.h"
#include "dev/watchdog.h"

#include <stdio.h> /* For printf() */

extern int rf_classify(void);
extern int nn_classify(void);

#define NUM_TESTS 10

void classify(void)
{
  rtimer_clock_t start = RTIMER_NOW();
  int i;
  int dummy = 0;
  for (i = 0; i < NUM_TESTS; ++i) {
#if 0
    dummy += nn_classify();
#else
    dummy += rf_classify();
#endif
    watchdog_periodic();
    //printf("test %d done\n", i);
  }
  rtimer_clock_t end = RTIMER_NOW();
  printf("for %u tests: %lu ticks, %lu ticks per second (result=%d)\n", NUM_TESTS,
      (uint32_t)(end - start), (uint32_t)RTIMER_ARCH_SECOND, dummy);
}

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "Node process");
AUTOSTART_PROCESSES(&node_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
//  static struct etimer timer;

  PROCESS_BEGIN();

  classify();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

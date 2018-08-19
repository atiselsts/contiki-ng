#include "contiki.h"
#include "dev/watchdog.h"
#include "iota/transfers.h"
#include <stdio.h>
#include <string.h>

#define NUM_INPUTS 1000
#if CONTIKI_TARGET_NATIVE
#define NUM_TESTS 10
#else
#define NUM_TESTS 1
#endif

PROCESS(iota_example_process, "IOTA");
AUTOSTART_PROCESSES(&iota_example_process);

// 81 ternary characters
char *seed = "HELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDD";

const char *initial_address = "TTMX9XI9ERCUWQASIUJFMPRRQETXHFLOSUOSQEBABFTHENORIUVDMHHDEIIUTPQTRAQAYOCAYHYD9OBLU";

//Define the output array, where the coins must go to.
TX_OUTPUT output_txs[] =
{{
//    .address = "TTMX9XI9ERCUWQASIUJFMPRRQETXHFLOSUOSQEBABFTHENORIUVDMHHDEIIUTPQTRAQAYOCAYHYD9OBLU",
    .value = 10000,
  }
};

PROCESS_THREAD(iota_example_process, ev, data)
{
  static struct etimer periodic_timer;

  PROCESS_BEGIN();
  etimer_set(&periodic_timer, CLOCK_SECOND);

  while(1) {
    // Define the transaction chars array. The char trytes will saved in this array. (base-27 encoded)
    static char transaction_chars[1][2673];
    int i, j;
    int security;
    rtimer_clock_t start, ticks;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

    for(security = 1; security <= 2; ++security) {
      printf("sec=%u\n", security);

      for(i = 0; i < NUM_INPUTS; ++i) {
        memcpy(output_txs[0].address, initial_address, 81);
        // Change the ASCII code slightly to obtain a different address.
        // This is done because the perfomance of the signing algorithm is input-data dependent.
        output_txs[0].address[i % 81] += (i + 80) / 81;

        start = RTIMER_NOW();
        for(j = 0; j < NUM_TESTS; ++j) {
          prepare_transfers(seed, security, output_txs, 1, NULL, 1, transaction_chars);
//          hw_watchdog_periodic();
        }
        ticks = RTIMER_NOW() - start;
        //printf("sec %u: %u ticks per %u tests\n", security, (unsigned int)ticks, NUM_TESTS);
        printf("%u\n", (unsigned int)ticks);
      }
    }

    //etimer_set(&periodic_timer, 10 * CLOCK_SECOND);
    printf("\n");
    break;
  }

 PROCESS_END();
}

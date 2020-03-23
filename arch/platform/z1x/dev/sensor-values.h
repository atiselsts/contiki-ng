#include "lib/random.h"

static inline uint16_t
random_number(uint16_t a, uint16_t b)
{
  uint16_t r = random_rand() % (b - a);
  return a + r;
}

#define SENSOR_VALUE_BATMON_TEMP random_number(0, 100)
#define SENSOR_VALUE_BATMON_VOLT random_number(100, 200)

#define SENSOR_VALUE_BMP_TEMP    random_number(200, 300)
#define SENSOR_VALUE_BMP_PRES    random_number(300, 400)

#define SENSOR_VALUE_HDC_TEMP    random_number(400, 500)
#define SENSOR_VALUE_HDC_HUM     random_number(500, 600)

#define SENSOR_VALUE_OPT7_LIGHT  random_number(600, 700)
#define SENSOR_VALUE_OPT8_LIGHT  random_number(700, 800)

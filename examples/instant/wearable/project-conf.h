#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

#include "../common-conf.h"

/* For the RPL + Orchestra experiments */
#define RPL_CONF_LEAF_ONLY 1

/* shorten the probing interval */
#ifndef RPL_CONF_PROBING_INTERVAL_MULTIPLIER
#define RPL_CONF_PROBING_INTERVAL_MULTIPLIER 10
#endif

#define RPL_CONF_PROBING_INTERVAL (RPL_CONF_PROBING_INTERVAL_MULTIPLIER * CLOCK_SECOND)

#define ROM_BOOTLOADER_ENABLE      0x0
#define BOARD_CONF_IOID_KEY_SELECT 0x17

/* #define LOG_CONF_LEVEL_RPL LOG_LEVEL_DBG */

#endif /* __PROJECT_CONF_H__ */

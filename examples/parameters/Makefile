CONTIKI_PROJECT = node
all: $(CONTIKI_PROJECT)

CONTIKI=../..

# slotframe settings
CFLAGS += -DTSCH_SCHEDULE_CONF_DEFAULT_LENGTH=7
CFLAGS += -DORCHESTRA_CONF_UNICAST_PERIOD=19

CFLAGS += -DSEND_INTERVAL_SEC=10

MAKE_MAC = MAKE_MAC_TSCH
MAKE_ROUTING = MAKE_ROUTING_RPL_CLASSIC

# 6tisch minimal
FIRMWARE_TYPE_6TISCH_MIN=1
# receiver based Orchestra + RPL
FIRMWARE_TYPE_ORCHESTRA_RB=2

ifndef FIRMWARE_TYPE
  FIRMWARE_TYPE := $(FIRMWARE_TYPE_6TISCH_MIN)
endif

ifneq ($(FIRMWARE_TYPE),$(FIRMWARE_TYPE_6TISCH_MIN))
# compile Orchestra
MODULES += os/services/orchestra
endif

# Energy usage estimation
MODULES += os/services/simple-energest

include $(CONTIKI)/Makefile.include

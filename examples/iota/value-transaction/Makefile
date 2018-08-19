TARGET=native

BOARD=launchpad/cc1310
CONTIKI=../../..

CONTIKI_PROJECT = main

WERROR=0

MODULES += os/services/iota
MODULES += os/services/keccak

MAKE_ROUTING = MAKE_ROUTING_NULLROUTING
MAKE_NET = MAKE_NET_NULLNET

all: $(CONTIKI_PROJECT)

include $(CONTIKI)/Makefile.include

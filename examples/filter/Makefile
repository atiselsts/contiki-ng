CONTIKI_PROJECT = filter
all: $(CONTIKI_PROJECT)

TARGET_LIBFILES += -lbutterworth
LDFLAGS += -L./

CFLAGS += -I/home/atis/sphere/feature/filters/butterworth/
CXXFLAGS += -std=c++11 -fpermissive

CUSTOM_LD = 1
LD = arm-none-eabi-g++

CUSTOM_RULE_LINK=1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

%.o: %.cpp
	$(TRACE_CC)
	$(Q)$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR_BOARD)/%.$(TARGET): %.o $(PROJECT_OBJECTFILES) $(PROJECT_LIBRARIES) $(CONTIKI_NG_TARGET_LIB)
	$(TRACE_LD)
	$(Q)$(CXX) $(LDFLAGS) $(TARGET_STARTFILES) ${filter-out %.a,$^} \
	    ${filter %.a,$^} $(TARGET_LIBFILES) -o $@

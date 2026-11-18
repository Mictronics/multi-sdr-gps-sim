HACKRFSDR ?= no
PLUTOSDR ?= no
OPT ?= -O2

DIALECT = -std=c11
CFLAGS += $(DIALECT) $(OPT) -g -W -Wall -Wextra -D_GNU_SOURCE -MMD -MP
LIBS = -lm -pthread -lcurl -lz -lpanel -lncurses
LDFLAGS =
SDR_OBJ = sdr_iqfile.o

ifeq ($(HACKRFSDR), yes)
    SDR_OBJ += sdr_hackrf.o
    CPPFLAGS += -DENABLE_HACKRFSDR
    CFLAGS += $(shell pkg-config --cflags libhackrf)
    LIBS_SDR += $(shell pkg-config --libs libhackrf)
endif

ifeq ($(PLUTOSDR), yes)
    SDR_OBJ += sdr_pluto.o
    CPPFLAGS += -DENABLE_PLUTOSDR
    CFLAGS += $(shell pkg-config --cflags libiio libad9361)
    LIBS_SDR += $(shell pkg-config --libs libiio libad9361)
endif

all: gps-sim

# Objects depend on the build flags, so changing HACKRFSDR/PLUTOSDR/OPT rebuilds them.
BUILD_FLAGS = .build-flags
$(BUILD_FLAGS): FORCE
	@echo '$(CC) $(CPPFLAGS) $(CFLAGS)' | cmp -s - $@ || echo '$(CC) $(CPPFLAGS) $(CFLAGS)' > $@

%.o: %.c $(BUILD_FLAGS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

gps-sim: fifo.o almanac.o gps.o gui.o sdr.o gps-sim.o $(SDR_OBJ)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR)

clean:
	rm -f *.o *.d $(BUILD_FLAGS) gps-sim

-include $(wildcard *.d)

.PHONY: all clean FORCE

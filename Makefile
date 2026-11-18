HACKRFSDR ?= no
PLUTOSDR ?= no
OPT ?= -O2
CPPFLAGS += -I.

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

# Unit tests, they need no SDR or terminal; gui and thread calls are stubbed.
TEST_BINS = tests/test_nav tests/test_rinex
TEST_COMMON = tests/stubs.o gps.o almanac.o fifo.o

tests/test_%: tests/test_%.o $(TEST_COMMON)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS)

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "== $$t"; ./$$t || exit 1; done

clean:
	rm -f *.o *.d tests/*.o tests/*.d $(BUILD_FLAGS) gps-sim $(TEST_BINS)

-include $(wildcard *.d tests/*.d)

.PHONY: all test clean FORCE

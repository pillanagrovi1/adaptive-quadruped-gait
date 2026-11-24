# Host build: shared library for Python + native test programs.
# Cortex-M4 build lives in m4/Makefile.

CC      ?= gcc
CFLAGS  ?= -O2 -std=c11 -Wall -Wextra -Wpedantic -Wshadow
INC      = -Ic/include -Ic/src -Ic/envs
SRC      = c/src/tr_agent.c c/src/tr_tabular.c c/src/tr_trace.c
HDR      = $(wildcard c/include/*.h c/src/*.h c/envs/*.h)
BUILD    = build

.PHONY: all test clean m4 m4-measure

all: $(BUILD)/libtinyrl.so $(BUILD)/corridor_run $(BUILD)/test_core

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/libtinyrl.so: $(SRC) $(HDR) | $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared $(INC) $(SRC) -o $@

$(BUILD)/corridor_run: c/tests/corridor_run.c $(SRC) $(HDR) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) c/tests/corridor_run.c $(SRC) -o $@

$(BUILD)/test_core: c/tests/test_core.c $(SRC) $(HDR) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) c/tests/test_core.c $(SRC) -lm -o $@

test: all
	./$(BUILD)/test_core
	python3 -m pytest -q python/tests

m4:
	$(MAKE) -C m4

m4-measure:
	python3 scripts/measure_m4.py

clean:
	rm -rf $(BUILD)
	$(MAKE) -C m4 clean

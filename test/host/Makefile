# Host test of the LACF 3.0 digipeater core with the PPM tables: make VECTORS=/path/to/lora-aprs-ideas/test
# FAST=0 tests the reference build_freq instead of the faster one the firmware uses (the output must be the same).
VECTORS ?= ../../../lora-aprs-ideas/test
FAST    ?= 1
CFLAGS  ?= -O2 -g -Wall -Wextra -fsanitize=address,undefined
DEFS    = -DD3_WITH_MODEL $(if $(filter 1,$(FAST)),-DHB_FAST)
SRC = test_digi3.c ../../src/digi3/digi3_core.c ../../lib/lacf3/hb3.c ../../lib/lacf3/hbi.c ../../lib/lacf3/hbc.c

test: test_digi3
	./test_digi3 $(VECTORS)

test_digi3: $(SRC) ../../src/digi3/digi3_core.h ../../lib/lacf3/hb3.h
	$(CC) $(CFLAGS) $(DEFS) -I../../src/digi3 -I../../lib/lacf3 -o $@ $(SRC) -lm

clean:
	rm -f test_digi3

.PHONY: test clean

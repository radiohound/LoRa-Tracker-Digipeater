# Host test of the LACF 3.0 digipeater core: make VECTORS=/path/to/lora-aprs-ideas/test
VECTORS ?= ../../../lora-aprs-ideas/test
CFLAGS  ?= -O2 -g -Wall -Wextra -fsanitize=address,undefined
SRC = test_digi3.c ../../src/digi3/digi3_core.c ../../src/digi3/hb3_nomodel.c ../../lib/lacf3/hb3.c

test: test_digi3
	./test_digi3 $(VECTORS)

test_digi3: $(SRC) ../../src/digi3/digi3_core.h ../../lib/lacf3/hb3.h
	$(CC) $(CFLAGS) -I../../src/digi3 -I../../lib/lacf3 -o $@ $(SRC) -lm

clean:
	rm -f test_digi3

.PHONY: test clean

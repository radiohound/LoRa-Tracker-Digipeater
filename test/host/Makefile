# PC tests of the tracker's settings engine (no Arduino, no radio)
CFLAGS ?= -O2 -g -Wall -Wextra -fsanitize=address,undefined
SRC = ../../src/settings/tk_settings.c

test: test_tk_settings test_flight_phase test-page
	./test_tk_settings
	./test_flight_phase

test_flight_phase: test_flight_phase.c ../../src/flight/flight_phase.c ../../src/flight/flight_phase.h
	$(CC) $(CFLAGS) -I../../src/flight -o $@ test_flight_phase.c ../../src/flight/flight_phase.c -lm

test_tk_settings: test_tk_settings.c $(SRC) ../../src/settings/tk_settings.h ../../include/config.h
	$(CC) $(CFLAGS) -I../../include -I../../src/settings -o $@ test_tk_settings.c $(SRC)

# the setup page (tools/setup.html) against a simulated tracker built from the same settings engine (needs node)
tks_sim: tks_sim.c $(SRC) ../../src/settings/tk_settings.h ../../include/config.h
	$(CC) -O1 -Wall -I../../include -I../../src/settings -o $@ tks_sim.c $(SRC)

test-page: tks_sim
	node test_setup_page.js ./tks_sim

clean:
	rm -rf test_flight_phase test_flight_phase.dSYM test_tk_settings test_tk_settings.dSYM tks_sim tks_sim.dSYM

.PHONY: test test-page clean

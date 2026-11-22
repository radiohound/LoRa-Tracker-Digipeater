// PC test helper: reads fields from argv, prints the encoded 32-byte frame as hex.
// usage: horus_v3_cli CALL SEQ TOD LAT LON ALT SPEED SATS ASCENT TEMP BATT [NOTE FRAMESIZE]   (use "-" to omit an optional)
#include "../src/horus/horus_v3.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
int main(int argc, char** argv) {
    if (argc != 12 && argc != 14) { fprintf(stderr, "need 11 or 13 arguments\n"); return 2; }
    HorusV3Packet p = {};
    p.callsign = argv[1]; p.sequence = atoi(argv[2]); p.timeOfDaySeconds = atoi(argv[3]);
    p.lat1e5 = atoi(argv[4]); p.lon1e5 = atoi(argv[5]); p.altitudeM = atoi(argv[6]);
    if (strcmp(argv[7], "-"))  { p.hasSpeed   = true; p.speedKmh        = atoi(argv[7]); }
    if (strcmp(argv[8], "-"))  { p.hasSats    = true; p.sats            = atoi(argv[8]); }
    if (strcmp(argv[9], "-"))  { p.hasAscent  = true; p.ascentCmPerS    = atoi(argv[9]); }
    if (strcmp(argv[10], "-")) { p.hasTemp    = true; p.tempInternalX10 = atoi(argv[10]); }
    if (strcmp(argv[11], "-")) { p.hasBattery = true; p.batteryMv       = atoi(argv[11]); }
    size_t size = 32;
    if (argc == 14) { p.note = argv[12]; size = (size_t)atoi(argv[13]); }     // the note argument may be empty
    uint8_t out[128];
    int n = horusV3Encode(p, out, size);
    if (n < 0) { printf("ERR %d\n", n); return 1; }
    for (int i = 0; i < n; i++) printf("%02X", out[i]);
    printf("\n");
    return 0;
}

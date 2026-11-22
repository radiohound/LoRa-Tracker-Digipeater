/* A simulated tracker for testing tools/setup.html without hardware: the real settings engine behind stdin/stdout,
 * with the same command handling as src/settings/tk_glue.cpp (dirty flag, apply before save, replies). */
#include "tk_settings.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>

static tk_settings_t cfg, applied;
static int dirty = 0;
static void emit(void *ctx, const char *l) { (void)ctx; printf("%s\r\n", l); }
static int radio_differs(void) {
    return cfg.freq_hz != applied.freq_hz || cfg.bw_hz != applied.bw_hz || cfg.sf != applied.sf || cfg.cr != applied.cr ||
           cfg.power_dbm != applied.power_dbm || cfg.sync != applied.sync || cfg.preamble != applied.preamble;
}
int main(int argc, char **argv) {
    char line[256];
    tks_defaults(&cfg); applied = cfg;
    if (argc > 1 && strcmp(argv[1], "n0call") != 0) { char fc[16]; snprintf(fc, sizeof fc, "set call %s", argv[1]); tks_exec(&cfg, fc, NULL, NULL); applied = cfg; }
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("[Boot] press a key within 3 s to change settings\r\n");
    while (fgets(line, sizeof line, stdin)) {
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = 0;
        if (!n) continue;
        if (!strcasecmp(line, "exit") || !strcasecmp(line, "run")) { printf("ok: running\r\n"); continue; }
        if (!strcasecmp(line, "bootloader")) {
            if (dirty) printf("there are unsaved changes: type save first, or reboot force to drop them\r\n");
            else printf("entering the bootloader (the firmware update takes over the serial port)...\r\n");
            continue;
        }
        unsigned fl = tks_exec(&cfg, line, emit, NULL);
        if (fl & TKS_CHANGED) { dirty = 1; }
        if (fl & TKS_APPLY) { applied = cfg; printf("ok: radio settings applied\r\n"); }
        if (fl & TKS_SAVE) {
            if (radio_differs()) printf("error: radio settings changed but not applied: type apply first\r\n");
            else { uint8_t rec[TKS_REC_MAX]; size_t m = tks_pack(&cfg, rec, sizeof rec); dirty = 0; printf("saved %lu bytes to flash\r\n", (unsigned long)m); }
        }
        if (fl & TKS_BEACON) printf("ok: beacon cycle starts now\r\n");
        if (fl & TKS_HORUS) printf("ok: a Horus packet follows the next beacon cycle, which starts now\r\n");
    }
    return 0;
}

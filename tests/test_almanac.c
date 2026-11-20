/* Checks the almanac download: source order, retries, validation and the cached file.
 * download_file is mocked, no network. Run with "make test".
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "gps.h"
#include "almanac.h"

static int failures;
static void expect(const char *what, long got, long want) {
    if (got != want) { printf("FAIL %-40s got %ld want %ld\n", what, got, want); failures++; }
}

static char sample[16384]; // tests/data/sample_almanac.al3 (NAVCEN, 32 satellites, 7 kB)
enum action { VALID, HTML, FEW, NOTFOUND, TIMEOUT };

// Scripted server: the action for each url, matched by substring; unlisted urls give 404
static struct { const char *match; enum action act; } script[8];
static char calls[32][256];
static int ncalls;

static void serve(int n, ...) {
    va_list ap;
    va_start(ap, n);
    memset(script, 0, sizeof (script));
    for (int i = 0; i < n; i++) {
        script[i].match = va_arg(ap, const char *);
        script[i].act = (enum action) va_arg(ap, int);
    }
    va_end(ap);
    ncalls = 0;
}

static void writeFile(const char *name, const char *text) {
    FILE *fp = fopen(name, "wb");
    fputs(text, fp);
    fclose(fp);
}

CURLcode download_file(const char *url, const char *filename, long timeout, const atomic_bool *cancel) {
    (void) cancel;
    expect("timeout per attempt [s]", timeout, 20);
    snprintf(calls[ncalls < 32 ? ncalls++ : 31], 256, "%s", url);
    enum action act = NOTFOUND;
    for (int i = 0; i < 8 && script[i].match; i++)
        if (strstr(url, script[i].match)) act = script[i].act;
    switch (act) {
        case VALID: writeFile(filename, sample); return CURLE_OK;
        case HTML: writeFile(filename, "<!DOCTYPE html>\n<html><body>Service unavailable</body></html>\n"); return CURLE_OK;
        case FEW: writeFile(filename, sample); { // the header announces 10 satellites
                FILE *fp = fopen(filename, "r+b"); fputs("10", fp); fclose(fp);
            } return CURLE_OK;
        case TIMEOUT: return CURLE_OPERATION_TIMEDOUT;
        default: return CURLE_HTTP_RETURNED_ERROR;
    }
}

static int validSats(void) {
    almanac_gps_t *a = almanac_init();
    if (almanac_read_file() != CURLE_OK) return 0;
    int n = 0;
    for (int sv = 0; sv < MAX_SAT; sv++) n += a->sv[sv].valid != 0;
    return n;
}

static void archiveUrl(char *url, int days_back) {
    time_t t = time(NULL) - (time_t) days_back * 86400;
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(url, 256, "navcen.uscg.gov/sites/default/files/gps/almanac/%Y/Sem/%j.al3", &tm);
}

int main(void) {
    atomic_bool cancel = false;
    char dir[] = "/tmp/test_almanac_XXXXXX", url[256];

    FILE *fp = fopen("tests/data/sample_almanac.al3", "rb");
    if (!fp || fread(sample, 1, sizeof (sample) - 1, fp) == 0) { printf("FAIL no sample almanac\n"); return 1; }
    fclose(fp);
    if (!mkdtemp(dir) || chdir(dir) != 0) { printf("FAIL no temp dir\n"); return 1; } // writes almanac.sem

    // First source valid: used, nothing else asked
    serve(1, "current_sem.al3", VALID);
    expect("NAVCEN current: result", almanac_download(&cancel), CURLE_OK);
    expect("NAVCEN current: requests", ncalls, 1);
    expect("NAVCEN current: satellites", validSats(), 32);

    // HTML error page with HTTP 200 is rejected, Celestrak next
    unlink(ALMANAC_FILE);
    serve(2, "current_sem.al3", HTML, "celestrak.org", VALID);
    expect("HTML page: result", almanac_download(&cancel), CURLE_OK);
    expect("HTML page: requests", ncalls, 2);
    expect("HTML page: Celestrak second", strstr(calls[1], "celestrak.org/GPS/almanac/SEM/almanac.sem.txt") != NULL, 1);
    expect("HTML page: satellites", validSats(), 32);

    // Timeout: 3 attempts (1 s and 3 s apart), then 404 on Celestrak, then the archive: today 404, yesterday valid
    unlink(ALMANAC_FILE);
    archiveUrl(url, 1);
    serve(2, "current_sem.al3", TIMEOUT, url, VALID);
    time_t t0 = time(NULL);
    expect("fallback: result", almanac_download(&cancel), CURLE_OK);
    long waited = (long) (time(NULL) - t0);
    expect("fallback: requests", ncalls, 6);
    expect("fallback: 3 attempts on NAVCEN", strstr(calls[2], "current_sem.al3") != NULL, 1);
    expect("fallback: backoff about 4 s", waited >= 3 && waited <= 6, 1);
    expect("fallback: Celestrak 4th", strstr(calls[3], "celestrak.org") != NULL, 1);
    archiveUrl(url, 0);
    expect("fallback: archive today 5th", strstr(calls[4], url) != NULL, 1);
    archiveUrl(url, 1);
    expect("fallback: archive yesterday 6th", strstr(calls[5], url) != NULL, 1);
    expect("fallback: satellites", validSats(), 32);

    // Too few satellites and an HTML page: rejected, archive all 404 (4 days), cached file kept
    serve(2, "current_sem.al3", FEW, "celestrak.org", HTML);
    expect("all fail, cache: result", almanac_download(&cancel), CURLE_OK);
    expect("all fail, cache: requests (2 + 4 days)", ncalls, 6);
    expect("all fail, cache: satellites", validSats(), 32);
    expect("all fail, cache: no temp file left", access(ALMANAC_FILE ".tmp", F_OK), -1);

    // Nothing cached and all fail: an error, no almanac
    unlink(ALMANAC_FILE);
    serve(0);
    expect("all fail, no cache: result", almanac_download(&cancel) != CURLE_OK, 1);
    expect("all fail, no cache: valid", almanac_init()->valid, 0);
    expect("all fail, no cache: no file", access(ALMANAC_FILE, F_OK), -1);

    // Cancelled during the backoff: stops without asking further sources
    serve(1, "current_sem.al3", TIMEOUT);
    cancel = true;
    expect("cancel: result", almanac_download(&cancel), CURLE_ABORTED_BY_CALLBACK);
    expect("cancel: requests", ncalls, 1);

    rmdir(dir);
    printf(failures ? "%d failures\n" : "all checks passed\n", failures);
    return failures != 0;
}

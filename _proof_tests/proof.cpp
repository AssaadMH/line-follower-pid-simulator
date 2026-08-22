// proof.cpp
// Host-side (PC) test harness for the SUIVEUR LASSAAD robot firmware.
// It copies the EXACT logic of two functions out of the .ino files and
// runs them on your PC so we can PROVE two bugs without any hardware.
//
//   Bug #1 : localCountLines()  (bb_sensors.ino) -> missing `return` on recursion
//   Bug #6 : compare()          (ba_util_fn.ino) -> 12-char patterns vs SensorCount=16
//
// Build:  g++ -std=c++11 -Wall -Wextra proof.cpp -o proof
// Run:    ./proof

#include <cstdio>
#include <cstdint>
#include <cstring>

static const int SensorCount = 16;          // same as firmware (a_pins_vars.ino)
uint16_t IntDsensors[SensorCount];           // firmware global, set per-test

// ----- tiny helper: load IntDsensors from a "0101.." string -----
void setSensors(const char* bits) {
    for (int i = 0; i < SensorCount; i++)
        IntDsensors[i] = (bits[i] == '1') ? 1 : 0;
}

// =====================================================================
//  ORIGINAL CODE  (copied verbatim from your .ino files)
// =====================================================================

// --- from ba_util_fn.ino ---
bool compare_ORIG(uint16_t a[], const char b[], int Length = SensorCount) {
    for (int i = 0; i < Length; i++) {
        if (b[i] == 'x') {
            // jump
        } else if (a[i] != (((uint16_t)b[i]) - 48)) { // -48 = ASCII '0'
            return false;
        }
    }
    return true;
}

// --- from bb_sensors.ino ---  (NOTE: recursive branches have NO `return`)
uint8_t localCountLines_ORIG(uint8_t cumulativeLineCounter = 0, int index = 0, int prevValue = 0) {
    if (index == SensorCount)
        return cumulativeLineCounter;
    else if (IntDsensors[index] == 1 && prevValue == 0)
        localCountLines_ORIG(cumulativeLineCounter + 1, index + 1, IntDsensors[index]); // <-- BUG: no return
    else
        localCountLines_ORIG(cumulativeLineCounter, index + 1, IntDsensors[index]);      // <-- BUG: no return
}

// =====================================================================
//  FIXED CODE
// =====================================================================

// Fix #6: compare only the characters that actually exist in the pattern.
bool compare_FIXED(uint16_t a[], const char b[], int Length) {
    for (int i = 0; i < Length; i++) {
        if (b[i] == 'x') continue;
        else if (a[i] != (uint16_t)(b[i] - '0')) return false;
    }
    return true;
}

// Fix #1: return the recursive results.
uint8_t localCountLines_FIXED(uint8_t cumulativeLineCounter = 0, int index = 0, int prevValue = 0) {
    if (index == SensorCount)
        return cumulativeLineCounter;
    else if (IntDsensors[index] == 1 && prevValue == 0)
        return localCountLines_FIXED(cumulativeLineCounter + 1, index + 1, IntDsensors[index]);
    else
        return localCountLines_FIXED(cumulativeLineCounter, index + 1, IntDsensors[index]);
}

// =====================================================================
//  TESTS
// =====================================================================
int passed = 0, failed = 0;
void check(const char* name, long got, long expected) {
    bool ok = (got == expected);
    printf("  [%s] %-46s got=%-6ld expected=%-6ld\n", ok ? "PASS" : "FAIL", name, got, expected);
    if (ok) passed++; else failed++;
}

int main() {
    printf("=====================================================\n");
    printf(" BUG #1  -- CountLines()  (bb_sensors.ino)\n");
    printf("=====================================================\n");
    setSensors("1100111001000000");
    printf("  sensor reading : 1100111001000000  (3 distinct line groups)\n");
    check("localCountLines_ORIG  (buggy)", localCountLines_ORIG(), 3);
    check("localCountLines_FIXED",         localCountLines_FIXED(), 3);

    setSensors("1010101000000000");
    printf("  sensor reading : 1010101000000000  (4 distinct line groups)\n");
    check("localCountLines_ORIG  (buggy)", localCountLines_ORIG(), 4);
    check("localCountLines_FIXED",         localCountLines_FIXED(), 4);

    printf("\n=====================================================\n");
    printf(" BUG #6  -- compare() pattern length vs SensorCount=16\n");
    printf("=====================================================\n");
    setSensors("1111111111110000");
    char pattern12[20] = "x1111111111x";   // 12 chars + zero padding (b[12..]=='\0')
    printf("  reading : 1111111111110000\n");
    printf("  pattern : \"x1111111111x\"  (12 chars, but SensorCount=16)\n");
    check("compare_ORIG  (default Length=16, buggy)", compare_ORIG(IntDsensors, pattern12) ? 1 : 0, 1);
    check("compare_FIXED (Length=strlen=12)",         compare_FIXED(IntDsensors, pattern12, (int)strlen(pattern12)) ? 1 : 0, 1);

    printf("\n-----------------------------------------------------\n");
    printf(" RESULT: %d passed, %d failed\n", passed, failed);
    printf(" (Every FAIL on an _ORIG line = a proven firmware bug.)\n");
    printf("-----------------------------------------------------\n");
    return 0;
}

// verify_fixed.cpp
// Mirrors the PATCHED firmware logic to confirm the fixes work and that the
// correctly-sized 16-char patterns were NOT broken by the compare() change.
// Build:  g++ -std=c++11 -Wall -Wextra verify_fixed.cpp -o verify_fixed
//   (should compile with ZERO warnings now)

#include <cstdio>
#include <cstdint>
#include <cstring>

static const int SensorCount = 16;
uint16_t IntDsensors[SensorCount];
void setSensors(const char* bits){ for(int i=0;i<SensorCount;i++) IntDsensors[i]=(bits[i]=='1')?1:0; }

// ---- PATCHED compare() (fix #6) ----
bool compare(uint16_t a[], const char b[], int Length = 0) {
    if (Length <= 0) Length = (int)strlen(b);
    if (Length > SensorCount) Length = SensorCount;
    for (int i = 0; i < Length; i++) {
        if (b[i] == 'x') { }
        else if (a[i] != (((uint16_t)b[i]) - 48)) return false;
    }
    return true;
}

// ---- PATCHED localCountLines() (fix #1) ----
uint8_t localCountLines(uint8_t c = 0, int index = 0, int prevValue = 0) {
    if (index == SensorCount) return c;
    else if (IntDsensors[index] == 1 && prevValue == 0)
        return localCountLines(c + 1, index + 1, IntDsensors[index]);
    else
        return localCountLines(c, index + 1, IntDsensors[index]);
}

int passed = 0, failed = 0;
void check(const char* name, long got, long expected) {
    bool ok = (got == expected);
    printf("  [%s] %-50s got=%-4ld expected=%-4ld\n", ok?"PASS":"FAIL", name, got, expected);
    if (ok) passed++; else failed++;
}

int main() {
    printf("== compare() : 12-char patterns now work ==\n");
    setSensors("1111111111110000");
    check("12-char pattern matches when it should", compare(IntDsensors, "x1111111111x") ? 1:0, 1);
    setSensors("1110111111110000");                       // sensor 3 broke the run of 1s
    check("12-char pattern rejects when it shouldn't match", compare(IntDsensors, "x1111111111x") ? 1:0, 0);

    printf("\n== compare() : 16-char patterns still work (not broken) ==\n");
    // "111111111xxxxx00" -> first 9 must be 1, last 2 must be 0, middle = don't care
    setSensors("1111111110101000");
    check("16-char otherconditions pattern matches", compare(IntDsensors, "111111111xxxxx00") ? 1:0, 1);
    setSensors("1111111110101011");                       // last 2 are 1 -> must reject
    check("16-char pattern rejects bad tail", compare(IntDsensors, "111111111xxxxx00") ? 1:0, 0);

    printf("\n== CountLines() : returns correct counts ==\n");
    setSensors("1100111001000000"); check("3 groups", localCountLines(), 3);
    setSensors("1010101000000000"); check("4 groups", localCountLines(), 4);
    setSensors("0000000000000000"); check("0 groups", localCountLines(), 0);
    setSensors("1111111111111111"); check("1 solid group", localCountLines(), 1);

    printf("\n----------------------------------------\n");
    printf(" RESULT: %d passed, %d failed\n", passed, failed);
    printf("----------------------------------------\n");
    return failed == 0 ? 0 : 1;
}

// lidar_calibration.ino
// Identify fixed mounting-post locations for POST_EXCLUSION_ZONES config.
//
// Keep robot completely stationary during 'a' runs.
//
// Serial commands (115200 baud, send with newline):
//   s       — single scan: print all non-zero bins
//   a       — average 20 scans + detect post clusters → suggested config
//   t<mm>   — set post-detection threshold (default 800 mm), e.g. t600
//   ?       — help

#include "YDLidarG2.h"

static const uint16_t LIDAR_MIN_MM = 150;
static const uint16_t LIDAR_MAX_MM = 10000;
static const uint16_t MIN_VALID    = 80;
static const uint8_t  N_AVG        = 20;

YDLidarG2 lidar(Serial1, 3);
uint16_t  closeThresh = 800;

// ---- scan helpers ----

bool takeScan(uint16_t out[360]) {
    uint32_t deadline = millis() + 2000;
    while (!lidar.update()) {
        if (millis() > deadline) { Serial.println(F("[scan timeout]")); return false; }
    }
    for (int i = 0; i < 360; i++) {
        uint16_t d = lidar.getDistance(i);
        out[i] = (d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM) ? d : 0;
    }
    return lidar.getValidBinCount() >= MIN_VALID;
}

void averageScans(uint16_t out[360]) {
    uint32_t sums[360] = {};
    uint8_t  hits[360] = {};
    uint16_t tmp[360];

    Serial.print(F("Collecting "));
    Serial.print(N_AVG);
    Serial.print(F(" scans "));

    for (int s = 0; s < N_AVG; s++) {
        if (takeScan(tmp)) {
            for (int i = 0; i < 360; i++) {
                if (tmp[i]) { sums[i] += tmp[i]; hits[i]++; }
            }
        }
        Serial.print('.');
    }
    Serial.println(F(" done"));

    for (int i = 0; i < 360; i++)
        out[i] = hits[i] ? (uint16_t)(sums[i] / hits[i]) : 0;
}

// ---- output helpers ----

// 72-char width overview: each char = 5 degrees
// Symbol key: # < 500mm  + 500–999mm  - 1000–1999mm  . >= 2000mm  _ no data
void printOverview(const uint16_t bins[360]) {
    Serial.println(F("\nOverview (each char = 5 deg, left=0 deg, right=355 deg)"));
    Serial.println(F("  # <0.5m   + 0.5-1m   - 1-2m   . >2m   _ no data"));
    Serial.print(F("  "));
    for (int g = 0; g < 72; g++) {
        uint16_t mn = 0xFFFF;
        for (int k = 0; k < 5; k++) {
            uint16_t d = bins[g * 5 + k];
            if (d && d < mn) mn = d;
        }
        char sym = '_';
        if (mn != 0xFFFF) {
            if      (mn < 500)  sym = '#';
            else if (mn < 1000) sym = '+';
            else if (mn < 2000) sym = '-';
            else                sym = '.';
        }
        Serial.print(sym);
        if ((g + 1) % 36 == 0) { Serial.println(); Serial.print(F("  ")); }
    }
    Serial.println();
}

void printScanTable(const uint16_t bins[360]) {
    Serial.println(F("\ndeg  dist_mm"));
    for (int i = 0; i < 360; i++) {
        if (!bins[i]) continue;
        char row[12];
        snprintf(row, sizeof(row), "%3d  %5u", i, bins[i]);
        Serial.println(row);
    }
}

void analyzePosts(const uint16_t bins[360]) {
    Serial.print(F("\nPost candidates (bins < "));
    Serial.print(closeThresh);
    Serial.println(F("mm):"));

    struct Cluster { int start, end; uint16_t minDist; };
    Cluster clusters[8];
    int  nClusters = 0;
    bool inCluster = false;

    for (int i = 0; i <= 360; i++) {
        bool isClose = (i < 360) && bins[i] > 0 && bins[i] < closeThresh;
        if (isClose && !inCluster) {
            inCluster = true;
            clusters[nClusters] = { i, i, bins[i] };
        } else if (isClose && inCluster) {
            clusters[nClusters].end = i;
            if (bins[i] < clusters[nClusters].minDist) clusters[nClusters].minDist = bins[i];
        } else if (!isClose && inCluster) {
            inCluster = false;
            if (++nClusters >= 8) break;
        }
    }

    if (!nClusters) {
        Serial.println(F("  None found. Try increasing threshold: t<mm>  e.g. t1200"));
        return;
    }

    Serial.println(F("  #   center   span   minDist_mm"));
    for (int i = 0; i < nClusters; i++) {
        int center = (clusters[i].start + clusters[i].end) / 2;
        int span   =  clusters[i].end - clusters[i].start + 1;
        char row[32];
        snprintf(row, sizeof(row), "  %d   %3d      %2d     %u",
                 i + 1, center, span, clusters[i].minDist);
        Serial.println(row);
    }

    Serial.println(F("\nSuggested — paste into Server/config_local.py:"));
    Serial.print(F("POST_EXCLUSION_ZONES = ["));
    for (int i = 0; i < nClusters; i++) {
        if (i) Serial.print(F(", "));
        int center    = (clusters[i].start + clusters[i].end) / 2;
        int halfWidth = (clusters[i].end - clusters[i].start) / 2 + 2;  // +2 deg margin
        char entry[12];
        snprintf(entry, sizeof(entry), "(%d, %d)", center, halfWidth);
        Serial.print(entry);
    }
    Serial.println(F("]"));
}

void printHelp() {
    Serial.println(F("\n=== LIDAR POST CALIBRATION ==="));
    Serial.println(F("Keep robot STATIONARY during 'a' runs."));
    Serial.println(F("Commands (115200 baud + newline):"));
    Serial.println(F("  s       — single scan table (all non-zero bins)"));
    Serial.println(F("  a       — average 20 scans + detect post clusters"));
    Serial.println(F("  t<mm>   — set close threshold, e.g. t600 or t1200"));
    Serial.println(F("  ?       — this help"));
    Serial.print  (F("Threshold: ")); Serial.print(closeThresh); Serial.println(F(" mm\n"));
}

// ---- setup / loop ----

void setup() {
    Serial.begin(115200);
    delay(500);
    if (!lidar.begin())     { Serial.println(F("ERROR: lidar.begin()")); while (1) {} }
    if (!lidar.startScan()) { Serial.println(F("ERROR: startScan()")); while (1) {} }
    delay(500);
    printHelp();
}

void loop() {
    static char    buf[16];
    static uint8_t bufLen = 0;

    lidar.update();  // drain Serial1 continuously

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[bufLen] = '\0';
            String cmd = String(buf);
            bufLen = 0;
            if (!cmd.length()) continue;

            if (cmd == "s") {
                Serial.println(F("\n--- SINGLE SCAN ---"));
                uint16_t bins[360];
                if (takeScan(bins)) {
                    Serial.print(F("Valid bins: ")); Serial.println(lidar.getValidBinCount());
                    printOverview(bins);
                    printScanTable(bins);
                }

            } else if (cmd == "a") {
                Serial.println(F("\n--- POST DETECTION ---"));
                uint16_t avg[360];
                averageScans(avg);
                printOverview(avg);
                analyzePosts(avg);

            } else if (cmd.startsWith("t")) {
                int v = cmd.substring(1).toInt();
                if (v > 0) {
                    closeThresh = (uint16_t)v;
                    Serial.print(F("Threshold set to ")); Serial.print(closeThresh); Serial.println(F(" mm"));
                } else {
                    Serial.println(F("Bad value. Example: t800"));
                }

            } else if (cmd == "?") {
                printHelp();
            } else {
                Serial.println(F("Unknown command. Send ? for help."));
            }
        } else {
            if (bufLen < sizeof(buf) - 1) buf[bufLen++] = c;
        }
    }
}

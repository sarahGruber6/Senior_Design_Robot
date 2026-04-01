#include "YDLidarG2.h"

// ------ Globals ------
YDLidarG2 lidar(Serial1, 3);
uint16_t distanceBins[360];

static const uint16_t LIDAR_MIN_MM = 150;
static const uint16_t LIDAR_MAX_MM = 10000;

bool scanning = false;
uint32_t publishSeq = 0;

// -------------------- Helpers --------------------

static bool isValidRangeMm(uint16_t d_mm) {
  return d_mm >= LIDAR_MIN_MM && d_mm <= LIDAR_MAX_MM;
}

void copyScanBinsFromLidar(uint16_t bins[360]) {
  for (int i = 0; i < 360; ++i) {
    uint16_t d = lidar.getDistance(i);
    bins[i] = isValidRangeMm(d) ? d : 0;
  }
}

// -------------------- LiDAR --------------------

void startLidar() {
  if (!lidar.begin()) {
    Serial.println("ERROR: lidar.begin() failed.");
    while (true) {}
  }

  if (!lidar.startScan()) {
    Serial.println("ERROR: startScan() failed.");
    while (true) {}
  }

  scanning = true;
  Serial.println("SCAN_START");
}

void exportScanLine(uint32_t nowMs, uint16_t validBins) {
  Serial.print("SCAN,");
  Serial.print(publishSeq++);
  Serial.print(",");
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(validBins);

  for (int deg = 0; deg < 360; ++deg) {
    Serial.print(",");
    Serial.print(distanceBins[deg]);
  }

  Serial.println();
}

void serviceLidar() {
  if (!scanning) return;

  if (lidar.update()) {
    uint32_t nowMs = millis();

    copyScanBinsFromLidar(distanceBins);
    uint16_t validBins = lidar.getValidBinCount();

    if (validBins >= 250) {
      exportScanLine(nowMs, validBins);
    }
  }
}

// -------------------- Setup / Loop --------------------

void setup() {
  Serial.begin(921600);
  delay(1000);

  Serial.println("BOOT");
  startLidar();
}

void loop() {
  serviceLidar();
}
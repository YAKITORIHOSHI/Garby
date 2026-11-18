#include "stubs/Arduino.h"
#include "stubs/Wire.h"
#include "../../BLE_Receiver-Final/MpuHeading.h"

int main() {
  // Owner's approximate floor observation: 100 physical degrees produced 90
  // MPU degrees. Exercise the real sensor reader, including signed integration.
  for (int dir : {1, -1}) {
    fakeNow = 1000;
    Wire = TwoWire{};
    MpuHeading mpu;
    if (!mpu.begin(8, 9)) return 2;
    mpu.requestReference();
    Wire.setGyroZ(dir * 1179);  // 9 raw degrees/s for a true 10 degrees/s
    fakeNow += 10;
    mpu.poll();
    for (int i = 0; i < 1000; ++i) {fakeNow += 10; mpu.poll();}
    if (!mpu.valid() || std::fabs(mpu.headingDeg() - dir * 100.0f) > .05f ||
        std::fabs(mpu.rateDps() - dir * 10.0f) > .001f) {
      std::fprintf(stderr, "FAIL 100-degree floor estimate corrects signed MPU angle and rate: %f/%f\n",
                   mpu.headingDeg(), mpu.rateDps());
      return 1;
    }
    std::printf("PASS %s MPU scale maps 90 raw degrees to the owner's estimated 100 degrees\n",
                dir > 0 ? "left" : "right");
  }
}

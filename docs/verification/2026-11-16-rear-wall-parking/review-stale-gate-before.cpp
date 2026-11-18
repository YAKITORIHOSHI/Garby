#define main rear_original_test_main
#include "/home/garby/SharedGarby/GARBY-CODEBASE/GarbyMostLatest-ForTesting/tests/firmware/test_rear_parking.cpp"
#undef main
int main() {
  fresh();
  plant.thetaDeg = 4.0;
  plant.gyroBiasDps = 0.1;
  plant.sendGo = false;
  const uint32_t start = fakeNow;
  uint32_t scanAt = fakeNow, commandAt = fakeNow, acq = 10000;
  double spinStartTheta = 1e8, maximumTurn = 0;
  tickHook = [&] {
    if (fakeNow - scanAt >= 174) { scanAt=fakeNow; rear(++acq, plant.thetaDeg); }
    if (fakeNow-commandAt >= 150) {
      commandAt=fakeNow;
      ESP_Serial.rx += fakeNow-start >= 120000 ? "GO\n" : "STOP:TURN\n";
    }
    if (a.isRunning() || b.isRunning()) {
      if (spinStartTheta > 1e7) spinStartTheta = plant.thetaDeg;
      maximumTurn=std::max(maximumTurn,std::fabs(plant.thetaDeg-spinStartTheta));
    }
    if (fakeNow-start>140000) resetQueued=true;
  };
  bool ok=backIntoPlace(2000);
  std::printf("ok=%d physical_theta=%+.3f maximum_spin=%+.3f gyro=%+.3f\n",ok,plant.thetaDeg,maximumTurn,plant.gyroDeg);
  std::puts(ESP_Serial.tx.c_str());
}

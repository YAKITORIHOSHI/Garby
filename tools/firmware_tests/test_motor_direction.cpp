// Which way the robot really travels: the real executor, through a model of the wiring.
//
// Routes and steering assume a rising step count means forward. On 2026-11-09 the
// owner confirmed that forward still went backward with the November 7 true/false DIR
// pair; the reported forward DIR levels below reproduce that. Which motor is the LEFT
// wheel is still not measured. Since 2026-11-11 named turns no longer depend on it:
// they are measured by the gyro, so a wrong guess is detected and swapped. What this
// cannot settle is the gyro's own sign (MPU mounted +Z up) or anything physical.
#pragma GCC optimize("O1")
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include "hallway_sim.h"

// The owner's 2026-11-09 observation: motor 1 rolls forward with DIR LOW, motor 2 with
// DIR HIGH. Independent of the production constants on purpose.
constexpr bool MOTOR1_ROLLS_FORWARD_ON_HIGH = false;
constexpr bool MOTOR2_ROLLS_FORWARD_ON_HIGH = true;
static float wheelSign(bool dirHighCountsUp, bool rollsForwardOnHigh) {
  return dirHighCountsUp == rollsForwardOnHigh ? 1.0f : -1.0f;
}

static FastAccelStepper wheel1, wheel2;
static hallway_sim::Plant plant;
static int failures = 0;
static void check(bool ok, const std::string& label) {
  if (ok) { std::printf("PASS %s\n", label.c_str()); return; }
  std::fprintf(stderr, "FAIL %s\n", label.c_str());
  failures++;
}
static float sign1 = 0, sign2 = 0;

static void freshRobot(bool motor1IsLeft = true) {
  tickHook = nullptr; backgroundHook = nullptr;
  wheel1 = FastAccelStepper{}; wheel2 = FastAccelStepper{};
  stepper1 = &wheel1; stepper2 = &wheel2;
  fullReset();
  garbyState = GarbyState::RUNNING;
  operatorSonarEnabled = false;
  heading = HeadingControl();
  ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  plant = hallway_sim::Plant();
  plant.motor1IsLeft = motor1IsLeft;
  plant.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [] { plant.tick(); };
  for (int i = 0; i < 400; ++i) { delay(1); pollESP(); }
}

static void bootsOnTheSamePinsWithForwardPolarity() {
  engine.connect = [](int pin) -> FastAccelStepper* {
    return pin == 21 ? &wheel1 : pin == 13 ? &wheel2 : nullptr;
  };
  setup();
  engine.connect = nullptr;
  check(stepper1 == &wheel1 && stepper2 == &wheel2,
        "setup connects motor 1 to STEP GPIO21 and motor 2 to STEP GPIO13");
  check(wheel1.dirPin == 22 && wheel2.dirPin == 12,
        "setup keeps motor 1 DIR on GPIO22 and motor 2 DIR on GPIO12: no wiring change");
  check(wheel1.dirHighCountsUp == MOTOR1_DIR_HIGH_COUNTS_UP &&
            wheel2.dirHighCountsUp == MOTOR2_DIR_HIGH_COUNTS_UP,
        "setup applies the named polarities");
  sign1 = wheelSign(wheel1.dirHighCountsUp, MOTOR1_ROLLS_FORWARD_ON_HIGH);
  sign2 = wheelSign(wheel2.dirHighCountsUp, MOTOR2_ROLLS_FORWARD_ON_HIGH);
  check(sign1 > 0 && sign2 > 0, "a rising step count rolls both wheels forward on the reported wiring");
  check(wheelSign(true, MOTOR1_ROLLS_FORWARD_ON_HIGH) < 0 && wheelSign(false, MOTOR2_ROLLS_FORWARD_ON_HIGH) < 0,
        "the November 7 true/false pair rolls both wheels backward: the reported fault");
  check(Serial.tx.find(std::string("[EXECUTOR BUILD:") + EXECUTOR_BUILD_ID +
                       " DIR1_UP=LOW DIR2_UP=HIGH TURNS=MPU]") != std::string::npos,
        "USB boot identifies this build, both direction polarities and MPU turns");
}

static void straightMovesGoForward() {
  for (const bool motor1IsLeft : {true, false}) {
    freshRobot(motor1IsLeft);
    check(beginRouteHeading() && safeMoveDistance(8000, false, false) &&
              wheel1.position * sign1 == 8000 && wheel2.position * sign2 == 8000 && plant.xCm > 0,
          std::string("a positive route leg moves forward with motor 1 as the ") +
              (motor1IsLeft ? "left" : "right") + " wheel");
  }
  freshRobot();
  check(safeMoveDistance(100, true, false) && wheel1.position * sign1 == 100 && wheel2.position * sign2 == 100,
        "a calibration bump moves forward, not backward");
  freshRobot();
  check(safeMoveDistance(-2000, false, false) && wheel1.position == -2000 && wheel2.position == -2000,
        "a negative move rolls backward without changing software target signs");
}

static void turnsGoTheNamedWay() {
  for (const bool motor1IsLeft : {true, false}) {
    const std::string side = motor1IsLeft ? "left" : "right";
    freshRobot(motor1IsLeft);
    beginRouteHeading();
    check(safeTurnLeft(90) && std::fabs(plant.thetaDeg - 90.0) < 1.5,
          "safeTurnLeft(90) rotates the chassis left with motor 1 as the " + side + " wheel");
    check(heading.motor1IsLeft() == motor1IsLeft && heading.turnSideConfirmed(),
          "the first MPU turn establishes that motor 1 is the " + side + " wheel");
    check(safeTurnRight(90) && std::fabs(plant.thetaDeg) < 1.5,
          "safeTurnRight(90) rotates the chassis back right with motor 1 as the " + side + " wheel");
    // Steering on the next straight uses the side the turn just proved.
    plant.enterHallway(-20.0);
    check(safeMoveDistance(120000, false, true) && std::fabs(plant.yCm) < 6.0,
          "wall centring steers toward the centre with motor 1 as the " + side + " wheel");
  }
}

// What the Start button does first, which is also the first thing to watch with the wheels lifted.
static void routeStartsForwardThenLeft() {
  freshRobot();
  double aheadAtTurn = 0;
  bool turned = false;
  tickHook = [&] {
    if (!turned && std::fabs(plant.thetaDeg) >= 80) {
      turned = true; aheadAtTurn = plant.xCm;
      resetQueued = true;
    }
  };
  runStart();
  tickHook = nullptr;
  check(turned && aheadAtTurn > 8000 / plant.stepsPerCm, "the real route starts by moving forward");
  check(turned && plant.thetaDeg > 0, "the unchanged route then turns left, measured by the MPU");
}

int main() {
  bootsOnTheSamePinsWithForwardPolarity();
  straightMovesGoForward();
  turnsGoTheNamedWay();
  routeStartsForwardThenLeft();
  if (failures) {
    std::fprintf(stderr, "%d checks failed\n", failures);
    return 1;
  }
}

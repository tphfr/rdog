#include <SimpleFOC.h>
#include <cmath>
#include <array>
#include <Ramp.h>

// ==============================
// --- HARDWARE INITIALIZATION ---
// ==============================

// I2C and Sensors
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Magnetic sensors (AS5600)
MagneticSensorI2C FRHsensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C FRKsensor = MagneticSensorI2C(AS5600_I2C);

// Drivers and Motors
BLDCDriver3PWM FRHdriver = BLDCDriver3PWM(32, 33, 25);
BLDCMotor FRHmotor = BLDCMotor(11);

BLDCDriver3PWM FRKdriver = BLDCDriver3PWM(26, 27, 14);
BLDCMotor FRKmotor = BLDCMotor(11);

// ==============================
// --- GLOBAL VARIABLES ---
// ==============================

double gearRatio = 36.0;
double hipLength = 0.15;
double thighLength = 0.21;
double kneeLength = 0.21;
double frontBackJointDistance = 0.6;
double leftRightJointDistance = 0.3;

double position[3] = {0, 0, 0};

ramp xreadings;
ramp yreadings;
ramp stepping;
ramp currentpitch;
ramp currentroll;
ramp currentyaw;

double pitchtheta = 30; // degrees
double rolltheta = 20;  // degrees
double yawtheta = 0;

double PRstate[1][2] = {0, 0};
double degAngles[3] = {0, 0, 0};

Commander command = Commander(Serial);
float target_velocity = 0;

// ==============================
// --- FOC SETUP ---
// ==============================

void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doVelP(char* cmd) { command.scalar(&FRHmotor.PID_velocity.P, cmd); }
void doVelI(char* cmd) { command.scalar(&FRHmotor.PID_velocity.I, cmd); }
void doVelD(char* cmd) { command.scalar(&FRHmotor.PID_velocity.D, cmd); }
void doAngleP(char* cmd) { command.scalar(&FRHmotor.P_angle.P, cmd); }

// ==============================
// --- KINEMATICS ---
// ==============================

std::array<double, 3> forwardKinematics(double theta1, double theta2, double theta3) {
  double t1 = theta1 * M_PI / 180.0;
  double t2 = theta2 * M_PI / 180.0;
  double t3 = theta3 * M_PI / 180.0;

  double r = (thighLength - kneeLength * cos(t3)) * sin(t2) + (kneeLength * sin(t3)) * cos(t2);
  double x = hipLength * sin(t1) - r * cos(t1);
  double y = -hipLength * cos(t1) - r * sin(t1);
  double z = -(thighLength - kneeLength * cos(t3)) * cos(t2) + (kneeLength * sin(t3)) * sin(t2);

  return {x, y, z};
}

std::array<double, 3> inverseKinematics(double x, double y, double z, bool isFrontLeg, bool isRightLeg) {
  double h = hipLength;
  double t = thighLength;
  double k = kneeLength;

  x += 0.15;
  y -= 0.3535;
  z += 0.0;

  y += (isFrontLeg ? 1 : -1) * (frontBackJointDistance / 2) * tan(currentpitch.update());
  y += (isRightLeg ? -1 : 1) * (leftRightJointDistance / 2) * tan(currentroll.update());

  double p = std::sqrt(std::max(x*x + y*y - h*h, 0.0));
  double L = std::sqrt(p*p + z*z);

  double theta1 = atan2(y, x);
  double cosTerm = (k*k - L*L - t*t) / (-2.0 * L * t);
  cosTerm = std::max(-1.0, std::min(1.0, cosTerm));
  double theta2 = M_PI/2.0 - std::acos(cosTerm) + atan2(z, p);

  double cosTheta3 = (L*L - k*k - t*t) / (-2.0 * k * t);
  cosTheta3 = std::max(-1.0, std::min(1.0, cosTheta3));
  double theta3 = std::acos(cosTheta3);

  theta2 += currentpitch.update();
  theta1 += (isRightLeg ? -1 : 1) * currentroll.update();

  theta1 *= gearRatio;
  theta2 *= gearRatio;
  theta3 *= gearRatio;

  return {theta1, theta2, theta3};
}

// ==============================
// --- PITCH/ROLL CONTROL ---
// ==============================

void updatePRstate(bool top, bool bottom, bool right, bool left) {
  if (top)    PRstate[0][0] = (PRstate[0][0] ==  1) ? 0 :  1;
  if (bottom) PRstate[0][0] = (PRstate[0][0] == -1) ? 0 : -1;
  if (right)  PRstate[0][1] = (PRstate[0][1] ==  1) ? 0 :  1;
  if (left)   PRstate[0][1] = (PRstate[0][1] == -1) ? 0 : -1;
}

void PRControl() {
  if (PRstate[0][0] != 0)
    currentpitch.go(PRstate[0][0] * pitchtheta * PI/180, 0.3);
  else
    currentpitch.go(0, 0.3);

  if (PRstate[0][1] != 0)
    currentroll.go(PRstate[0][1] * rolltheta * PI/180, 0.3);
  else
    currentroll.go(0, 0.3);
}

// ==============================
// --- STEP MOTION ---
// ==============================

void stepMotion() {
  static double stepHeight = 0.075;
  static double stepLength = 4 * stepHeight;

  double x1 = 0;
  double y1 = 1;
  xreadings.go(x1, 0.5);
  yreadings.go(y1, 0.5);

  double maxDirectionalMagnitude = 1;
  double directionalAngle = atan2(xreadings.update(), yreadings.update());
  double directionalMagnitudeCoefficient =
      sqrt(xreadings.update()*xreadings.update() + yreadings.update()*yreadings.update()) / maxDirectionalMagnitude;

  static double xyz[3] = {0, 0, 0};
  static unsigned long lastLoop = millis();
  double time = (millis() - lastLoop) / 1000.0;
  static double halfloopTime = 3.0;
  static double returnTime = 3.0;

  if (time > halfloopTime) {
    xyz[1] = 0;
    xyz[2] = stepLength*0.5*cos((time-halfloopTime)/halfloopTime*PI);
  } else {
    xyz[1] = stepHeight*sin(time/halfloopTime*PI);
    xyz[2] = -stepLength*0.5*cos(time/halfloopTime*PI);
  }
  if (time > halfloopTime + returnTime) lastLoop = millis();

  double x = xyz[2] * directionalMagnitudeCoefficient * sin(directionalAngle);
  double z = xyz[2] * directionalMagnitudeCoefficient * cos(directionalAngle);
  
  std::array<double, 3> moveTarget = inverseKinematics(x, xyz[1], z, true, true);
  //Serial.println("coords" + String(x) + "," + String(xyz[1]) + "," + String(z) + " angles " + String(moveTarget[0]) + "," + String(moveTarget[1]) + "," + String(moveTarget[2]));

  // if you have knee motor: FRTmotor.move(moveTarget[0]);
  FRHmotor.move(moveTarget[1]);
  //FRKmotor.move(moveTarget[2]);
}

// ==============================
// --- SERIAL INTERFACE ---
// ==============================

void processSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("T")) {
      input.remove(0, 1);
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        double theta1 = input.substring(0, firstComma).toDouble();
        double theta2 = input.substring(firstComma + 1, secondComma).toDouble();
        double theta3 = input.substring(secondComma + 1).toDouble();
        auto coords = forwardKinematics(theta1, theta2, theta3);
        Serial.printf("X: %.3f Y: %.3f Z: %.3f\n", coords[0], coords[1], coords[2]);
      } else Serial.println("Invalid input. Use: Ttheta1,theta2,theta3");
    } else {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        double x = input.substring(0, firstComma).toDouble();
        double y = input.substring(firstComma + 1, secondComma).toDouble();
        double z = input.substring(secondComma + 1).toDouble();
        auto angles = inverseKinematics(x, y, z, true, true);
        Serial.printf("Angles: %.3f, %.3f, %.3f\n", angles[0], angles[1], angles[2]);
      } else Serial.println("Invalid input. Use: x,y,z");
    }
  }
}

// ==============================
// --- SETUP & LOOP ---
// ==============================

void setup() {
  delay(400);
  Serial.begin(9600);
  Serial.println("Inverse Kinematics + FOC Ready");

  // I2C setup
  I2Cone.begin(19, 18, 400000);
  //I2Ctwo.begin(23, 5, 400000);

  FRHsensor.init(&I2Cone);
  //FRKsensor.init(&I2Ctwo);

  FRHmotor.linkSensor(&FRHsensor);
  //FRKmotor.linkSensor(&FRKsensor);

  FRHdriver.voltage_power_supply = 9;
  FRKdriver.voltage_power_supply = 9;
  FRHdriver.pwm_frequency = 50000;
  FRKdriver.pwm_frequency = 50000;
  FRHdriver.init();
  //FRKdriver.init();

  FRHmotor.linkDriver(&FRHdriver);
  //FRKmotor.linkDriver(&FRKdriver);

  FRHmotor.controller = MotionControlType::angle;
  FRKmotor.controller = MotionControlType::angle;

  FRHmotor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  FRKmotor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  FRHmotor.voltage_limit = 3;
  FRKmotor.voltage_limit = 3;
  FRHmotor.current_limit = 10;
  FRKmotor.current_limit = 10;
  FRHmotor.velocity_limit = 100;
  FRKmotor.velocity_limit = 100;

  FRHmotor.PID_velocity.P = 0.05;
  FRKmotor.PID_velocity.P = 0.05;
  FRHmotor.PID_velocity.I = 0.4;
  FRKmotor.PID_velocity.I = 0.4;
  FRHmotor.P_angle.P = 1;
  FRKmotor.P_angle.P = 1;
  FRHmotor.LPF_velocity.Tf = 0.01;
  FRKmotor.LPF_velocity.Tf = 0.01;

  FRHmotor.init();
  //FRKmotor.init();
  FRHmotor.initFOC();
  //FRKmotor.initFOC();

  command.add('T', doTarget, "target velocity");
  command.add('P', doVelP, "velocity P");
  command.add('I', doVelI, "velocity I");
  command.add('D', doVelD, "velocity D");
  command.add('A', doAngleP, "angle P");

  Serial.println("Motor ready. Enter commands or x,y,z values.");


  FRHmotor.useMonitoring(Serial);

  // ===========================
  // ANGLE SETUP BUTTON HERE 👇
  // ===========================
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), angleSetup, RISING);
  // Define `void angleSetup()` to calibrate zero positions.
}

bool kill_switch = true; // true = enabled, false = disabled
void checkKillSwitch() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '/') { kill_switch = !kill_switch;
        Serial.print("Kill switch: ");
        Serial.println(kill_switch ? "ON" : "OFF"); } } }

void loop() {
  checkKillSwitch(); if (kill_switch) { return; }
  FRHmotor.loopFOC();
  //FRKmotor.loopFOC();

  //processSerialInput();
  //stepMotion();
  FRHmotor.move(target_velocity);
  command.run();
}

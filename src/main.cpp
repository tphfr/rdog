// MKS DUAL FOC open-loop speed control routine.Test library: SimpleFOC 2.1.1 Test hardware: MKS DUAL FOC V3.1
// Enter "T+number" in the serial port to set the speed of the two motors. For example, set the motor to rotate at 10rad/s, input "T10", and the motor will rotate at 5rad/s by default when it is powered on
// When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7), and set it to your own number of pole pairs
// The default power supply voltage set by the program is 12V, please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply

#include <SimpleFOC.h>
#include <cmath>
#include <tuple>

double position[3] = {0, 0, 0};

//distance from hip's rotation axis to the knee driver gear, all in meters
double hipLength = 0.15;
double thighLength = 0.25;
double kneeLength = 0.25;

double degAngles[3] = {0, 0, 0};

BLDCMotor FRHmotor = BLDCMotor(11);
BLDCDriver3PWM FRHdriver = BLDCDriver3PWM(32,33,25);

BLDCMotor FRTmotor = BLDCMotor(11);
BLDCDriver3PWM FRTdriver  = BLDCDriver3PWM(26,27,14);

BLDCMotor FRKmotor = BLDCMotor(11);
BLDCDriver3PWM FRKdriver  = BLDCDriver3PWM(26,27,14);

// Constants: t, h, k
// Input: x, y, z
// Output: theta1, theta2, theta3 (in degrees, shifted origin)

std::tuple<double,double,double> inverseKinematics(double x, double y, double z) {
    // Step 1: compute r and theta1
    double r = sqrt(x*x + y*y);
    double theta1 = atan2(x, -y) * 180.0 / M_PI; // degrees

    // Step 2: solve for theta3 from law of cosines
    double d = sqrt(x*x + y*y) - hipLength;
    double D = (thighLength*thighLength + kneeLength*kneeLength - d*d) / (2*thighLength*kneeLength);
    if(D > 1) D = 1; 
    if(D < -1) D = -1;
    double theta3 = acos(D) * 180.0 / M_PI; // degrees

    // Step 3: solve for theta2 from z-equation
    double A = thighLength - kneeLength * cos(theta3 * M_PI/180.0);
    double B = kneeLength * sin(theta3 * M_PI/180.0);
    double theta2 = atan2(z - A*cos(0), B) * 180.0 / M_PI; // simplified form

    // Step 4: apply origin shift (θ1=90, θ2=40, θ3=85 as zero reference)
    theta1 -= 90.0;
    theta2 -= 40.0;
    theta3 -= 85.0;

    return {theta1, theta2, theta3};
}


void setup () {
  Serial.begin(9600);
  Serial.println("Inverse Kinematics Ready");
  Serial.println(std::get<0>(inverseKinematics(0,0,0)));
  Serial.println(std::get<1>(inverseKinematics(0,0,0)));
  Serial.println(std::get<2>(inverseKinematics(0,0,0)));
}
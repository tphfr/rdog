#include <Arduino.h>
#include <Ramp.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SimpleFOC.h>
#include <cmath>
#include <array>

int id;

MagneticSensorI2C RIGHTsensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C LEFTsensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor parameters
BLDCMotor RIGHTmotor = BLDCMotor(11);
BLDCDriver3PWM RIGHTdriver = BLDCDriver3PWM(26,27,14);

BLDCMotor LEFTmotor = BLDCMotor(11);
BLDCDriver3PWM LEFTdriver = BLDCDriver3PWM(32,33,25);

//Command settings

uint8_t broadcastAddress[] = {0xB0, 0xCB, 0xD8, 0xEE, 0x59, 0x74};

float thetaR = 0;
float thetaL = 0;

float zeroOffsetR = 0;
float zeroOffsetL = 0;

typedef struct {
  uint8_t command;
  float thetar;
  float thetal;
} ControlPacket;

enum states : uint8_t {
  STATE_IDLE = 0,
  STATE_WALKING = 1,
  STATE_TURNING = 2
};

uint8_t state = STATE_IDLE;

enum ControlCommand : uint8_t {
  CMD_START = 0,
  CMD_STOP = 1,
  CMD_INIT = 2,
  CMD_CALIBRATE = 3,
  CMD_NONE = 99
};

ControlPacket incomingInformation;

void addMaster() {
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;     // must match receiver WiFi channel
  peerInfo.encrypt = false;

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.print("Failed to add peer ");
    }
  
}



void initialize() {

  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  LEFTsensor.init(&I2Cone);
  RIGHTsensor.init(&I2Ctwo);
  //Connect the motor object with the sensor object
  LEFTmotor.linkSensor(&LEFTsensor);
  RIGHTmotor.linkSensor(&RIGHTsensor);

  //Supply voltage setting [V]
  RIGHTdriver.pwm_frequency = 50000;
  RIGHTdriver.voltage_power_supply = 9;
  RIGHTdriver.init();

  LEFTdriver.pwm_frequency = 50000;
  LEFTdriver.voltage_power_supply = 9;
  LEFTdriver.init();

  //Connect the motor and driver objects
  LEFTmotor.linkDriver(&LEFTdriver);
  RIGHTmotor.linkDriver(&RIGHTdriver);

  //FOC model selection
  LEFTmotor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  RIGHTmotor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  LEFTmotor.controller = MotionControlType::angle;
  RIGHTmotor.controller = MotionControlType::angle;

  //Speed PI loop setting
  LEFTmotor.PID_velocity.P = 0.1;
  RIGHTmotor.PID_velocity.P = 0.1;
  LEFTmotor.PID_velocity.I = 1;
  RIGHTmotor.PID_velocity.I = 1;

  //Angle P ring setting
  LEFTmotor.P_angle.P = 1;
  RIGHTmotor.P_angle.P = 1;

  //Speed low-pass filter time constant
  LEFTmotor.LPF_velocity.Tf = 0.01;
  RIGHTmotor.LPF_velocity.Tf = 0.01;
  
  //Max motor limit motor
  LEFTmotor.voltage_limit = 3;
  LEFTmotor.current_limit = 6;
  RIGHTmotor.voltage_limit = 3;
  RIGHTmotor.current_limit = 6;

  //Set a maximum speed limit
  LEFTmotor.velocity_limit = 80;
  RIGHTmotor.velocity_limit = 80;

  LEFTmotor.useMonitoring(Serial);
  RIGHTmotor.useMonitoring(Serial);

  
  //Initialize the motor
  LEFTmotor.init();
  RIGHTmotor.init();
  //Initialize FOC
  LEFTmotor.initFOC();
  RIGHTmotor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingInformation, incomingData, sizeof(incomingInformation));
  
  if (incomingInformation.command == CMD_INIT) {
    if (incomingInformation.thetal == 10000) {
      id = 0;
    } else if (incomingInformation.thetal == 10001) {
      id = 1;
    } else if (incomingInformation.thetal == 10002) {
      id = 2;
    } else if (incomingInformation.thetal == 10003) {
      id = 3;
    } else if (incomingInformation.thetal == 10004) {
      id = 4;
    } else if (incomingInformation.thetal == 10005) {
      id = 5;
    }
        
    initialize();
    return;
  }

  if (incomingInformation.command == CMD_CALIBRATE) {
    //updateZero();
    return;
  }

  if (incomingInformation.command == CMD_START) {
    state = STATE_WALKING;
    return;
  }

  if (incomingInformation.command == CMD_STOP) {
    state = STATE_IDLE;
    return;
  }

  thetaL = incomingInformation.thetal;
  thetaR = incomingInformation.thetar;

}

void setup() {
  Serial.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register receive callback
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  addMaster();
  
  initialize();
}

void loop() {

}
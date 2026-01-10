#include <Arduino.h>
#include <Ramp.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SimpleFOC.h>
#include <cmath>
#include <array>
#include <PSX.h>

#define DATA_PIN 12
#define CMD_PIN 14
#define ATT_PIN 13
#define CLOCK_PIN 15

PSX psx;

PSX::PSXDATA PSXdata;
int PSXerror;

//receiver MAC Address

uint8_t peerMac[6][6] = {
  {0xB0, 0xCB, 0xD8, 0xEE, 0x72, 0x08}, // Front-Upper
  {0xB0, 0xCB, 0xD8, 0xEE, 0x71, 0x88}, // Front-Middle
  {0xB0, 0xCB, 0xD8, 0xEE, 0x64, 0xD0}, // Front-Lower
  {0xB0, 0xCB, 0xD8, 0xEE, 0x5d, 0x24}, // Back-Upper
  {0xD4, 0xE9, 0xF4, 0x72, 0xFA, 0x10}, // Back-Middle
  {0xB0, 0xCB, 0xD8, 0xEE, 0x5C, 0xAC}  // Back-Lower
};

enum states : uint8_t {
  STATE_IDLE = 0,
  STATE_WALKING = 1,
  STATE_TURNING = 2
};

uint8_t state = STATE_IDLE;

enum ControlCommand : uint8_t {
  CMD_NONE = 0,
  CMD_STOP = 1,
  CMD_INIT = 2,
  CMD_CALIBRATE = 3
};

typedef struct {
  uint8_t command;
  float thetal;
  float thetar;
} ControlPacket;

ControlPacket outgoingInformation;
ControlPacket incomingInformation;

String success;

// ==============================
// --- GLOBAL VARIABLES ---
// ==============================

float gearRatio = 36.0;
float hipLength = 0.15;
float thighLength = 0.21;
float kneeLength = 0.21;
float frontBackJointDistance = 0.6;
float leftRightJointDistance = 0.3;
float magnitude = 0.0;
ramp velocityRamp[2]; // Ramp for smooth transitions

float maxAngleSpeed = 90; // degrees per second

float tiltState[2] = {0, 0}; // [0] = pitch (i/k), [1] = roll (j/l)
float pitchTheta = 30; // degrees
float rollTheta = 20;  // degrees
ramp currentPitch;
ramp currentRoll;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // milliseconds

float langle;
float lagnitude;

float rangle;
float ragnitude;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingInformation, incomingData, sizeof(incomingInformation));
  
}

std::array<float, 3> inverseKinematics(float x, float y, float z, bool isFrontLeg, bool isRightLeg) {
  double h = hipLength;
  double t = thighLength;
  double k = kneeLength;

  x += 0.15;
  y -= 0.296984848098;
  z += 0.0;

  //y += (isFrontLeg ? 1 : -1) * (frontBackJointDistance / 2) * tan(currentpitch.update());
  //y += (isRightLeg ? -1 : 1) * (leftRightJointDistance / 2) * tan(currentroll.update());

  double p = std::sqrt(std::max(x*x + y*y - h*h, 0.0));
  double L = std::sqrt(p*p + z*z);

  double theta1 = atan2(abs(x), abs(y)) + atan2(p, h);
  double cosTerm = (k*k - L*L - t*t) / (-2.0 * L * t);
  cosTerm = std::max(-1.0, std::min(1.0, cosTerm));
  double theta2 = M_PI/2.0 - std::acos(cosTerm) + atan2(z, p);

  double cosTheta3 = (L*L - k*k - t*t) / (-2.0 * k * t);
  cosTheta3 = std::max(-1.0, std::min(1.0, cosTheta3));
  double theta3 = std::acos(cosTheta3);

  //theta2 += currentpitch.update();
  //theta1 += (isRightLeg ? -1 : 1) * currentroll.update();

  theta1 *= (180.0 / M_PI);
  theta2 *= (180.0 / M_PI);
  theta3 *= (180.0 / M_PI);

  return {float(theta1), float(theta2), float(theta3)};
}

std::array<float, 12> calculateAngles() {
  float thetafr1, thetafr2, thetafr3;
  float thetafl1, thetafl2, thetafl3;
  float backright1, backright2, backright3;
  float backleft1, backleft2, backleft3;
  
  float stepCycleTime = 1000.0; // milliseconds
  

  return {thetafr1, thetafr2, thetafr3,
          thetafl1, thetafl2, thetafl3,
          backright1, backright2, backright3,
          backleft1, backleft2, backleft3};
}

void addPeers() {
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;     // must match receiver WiFi channel
  peerInfo.encrypt = false;

  for (int i = 0; i <= 5; i++) {
    memcpy(peerInfo.peer_addr, peerMac[i], 6);

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.print("Failed to add peer ");
      Serial.println(i);
    }
  }
}

void sendData(std::array<std::array<float, 3>, 4> coordinates) {
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    std::array<float, 12> angles = calculateAngles();
    for (int i = 0; i <= 5; i++) {
      if (i <= 2 ) {
        outgoingInformation.thetar = coordinates[0][i]; // Front Right
        outgoingInformation.thetal = coordinates[1][i]; // Front Left
      } else {
        outgoingInformation.thetar = coordinates[2][i-3]; // Back Right
        outgoingInformation.thetal = coordinates[3][i-3]; // Back Left
      }
      esp_now_send(peerMac[i],(uint8_t*)&outgoingInformation,sizeof(outgoingInformation));
    }
  }
}

void processPSData() {

  PSXerror = psx.read(PSXdata);
  
  if(PSXdata.buttons & PSXBTN_START) {
    Serial.println("Start button pressed");
    outgoingInformation.command = CMD_INIT;
    esp_now_send((uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF",(uint8_t*)&outgoingInformation,sizeof(outgoingInformation));
    outgoingInformation.command = CMD_NONE;
  }

  //read data
  float LeftX = PSXdata.JoyLeftX; // 0-2
  float LeftY = PSXdata.JoyLeftY;
  float RightX = PSXdata.JoyRightX;
  float RightY = PSXdata.JoyRightY;
  //if not touching center
  float lerpTime = 200; //time to lerp to new value

  static int lastLX = 128;
  static int lastLY = 128;

  if (abs(LeftX - lastLX) > 2) {
     velocityRamp[0].go(LeftX, 1000);
    lastLX = LeftX;
  }

  if (abs(LeftY - lastLY) > 2) {
   velocityRamp[1].go(LeftY, 1000);
    lastLY = LeftY;
  }

  

   //go to -1 to 1
    float vr0 = (velocityRamp[0].update()-128.0)/128.0;
    float vr1 = -(velocityRamp[1].update()-127.0)/128.0;

    //constrain

    //from square to circle
    float xc = vr0 * sqrt(1.0 - (vr1*vr1)/2.0);
    float yc = vr1 * sqrt(1.0 - (vr0*vr0)/2.0);
    //theta, magnitude
    langle = atan2(yc, xc) - M_PI/2.0;  // Rotate so forward (up) is 0 degrees
    langle = fmod(langle + 2.0 * M_PI, 2.0 * M_PI);  // Make all positive (0 to 2π)
    lagnitude = sqrt(xc*xc + yc*yc);  // 0 → 1
    if (lagnitude == 0) {
      langle = 0; // If no magnitude, set angle to 0 to avoid undefined angle
    }

    Serial.println("langle: " + String(langle) + " lagnitude: " + String(lagnitude));

}

std::array<std::array<float, 3>, 4> stepMotion() {
  static float stepHeight = 0.075;
  static float stepLength = 4 * stepHeight;

  static float xyzFR[3] = {0, 0, 0};
  static float xyzFL[3] = {0, 0, 0};
  static float xyzBR[3] = {0, 0, 0};
  static float xyzBL[3] = {0, 0, 0};
  static unsigned long lastLoopFR = millis();
  static unsigned long lastLoopFL = millis();
  static unsigned long lastLoopBR = millis();
  static unsigned long lastLoopBL = millis();
  float timeFR = (millis() - lastLoopFR) / 1000.0;
  float timeFL = (millis() - lastLoopFL) / 1000.0;
  float timeBR = (millis() - lastLoopBR) / 1000.0;
  float timeBL = (millis() - lastLoopBL) / 1000.0;
  static float halfloopTime = 3.0;
  static float returnTime = 3.0;

  if (timeFR > halfloopTime) {
    xyzFR[1] = 0;
    xyzFR[2] = stepLength*0.5*cos((timeFR-halfloopTime)/returnTime*PI);
  } else {
    xyzFR[1] = stepHeight*sin(timeFR/halfloopTime*PI);
    xyzFR[2] = -stepLength*0.5*cos(timeFR/halfloopTime*PI);
  }
  if (timeFR > halfloopTime + returnTime) lastLoopFR = millis();

  if (timeFL > halfloopTime) {
    xyzFL[1] = stepHeight*sin(timeFL/halfloopTime*PI);
    xyzFL[2] = -stepLength*0.5*cos(timeFL/halfloopTime*PI);
  } else {
    xyzFL[1] = 0;
    xyzFL[2] = stepLength*0.5*cos((timeFL-halfloopTime)/returnTime*PI);
    
  }
  if (timeFL > halfloopTime + returnTime) lastLoopFL = millis();

  if (timeBR > halfloopTime) {
    xyzBR[1] = 0;
    xyzBR[2] = stepLength*0.5*cos((timeBR-halfloopTime)/returnTime*PI);
  } else {
    xyzBR[1] = stepHeight*sin(timeBR/halfloopTime*PI);
    xyzBR[2] = -stepLength*0.5*cos(timeBR/halfloopTime*PI);
  }
  if (timeBR > halfloopTime + returnTime) lastLoopBR = millis();

  if (timeBL > halfloopTime) {
    xyzBL[1] = stepHeight*sin(timeBL/halfloopTime*PI);
    xyzBL[2] = -stepLength*0.5*cos(timeBL/halfloopTime*PI);
  } else {
    xyzBL[1] = 0;
    xyzBL[2] = stepLength*0.5*cos((timeBL-halfloopTime)/returnTime*PI);
    
  }
  if (timeBL > halfloopTime + returnTime) lastLoopBL = millis();


  xyzFR[0] = xyzFR[0]; //* lagnitude * sin(langle);
  xyzFR[2] = xyzFR[2]; //* lagnitude * cos(langle);

  xyzFL[0] = xyzFL[0]; //* lagnitude * sin(langle);
  xyzFL[2] = xyzFL[2]; //* lagnitude * cos(langle);

  xyzBR[0] = xyzBR[0]; //* lagnitude * sin(langle);
  xyzBR[2] = xyzBR[2]; //* lagnitude * cos(langle);

  xyzBL[0] = xyzBL[0]; //* lagnitude * sin(langle);
  xyzBL[2] = xyzBL[2]; //* lagnitude * cos(langle);

  std::array<float, 3> moveTargetFR = inverseKinematics(xyzFR[0], xyzFR[1], xyzFR[2], true, true);
  std::array<float, 3> moveTargetFL = inverseKinematics(xyzFL[0], xyzFL[1], xyzFL[2], true, false);
  std::array<float, 3> moveTargetBR = inverseKinematics(xyzBR[0], xyzBR[1], xyzBR[2], false, true);
  std::array<float, 3> moveTargetBL = inverseKinematics(xyzBL[0], xyzBL[1], xyzBL[2], false, false);

  return {moveTargetFR, moveTargetFL, moveTargetBR, moveTargetBL};
  //Serial.println("coords" + String(x) + "," + String(xyz[1]) + "," + String(z) + " angles " + String(moveTarget[0]) + "," + String(moveTarget[1]) + "," + String(moveTarget[2]));
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  addPeers();

  psx.setupPins(DATA_PIN, CMD_PIN, ATT_PIN, CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);

  velocityRamp[0].go(128);
  velocityRamp[1].go(127);
  return;

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  


  }


void loop() {
  delay(100);

  //processSerialInput();
  processPSData();

  if (state == STATE_IDLE) {
    return;
  } else if (state == STATE_WALKING) {
    sendData(stepMotion());
  } else if (state == STATE_TURNING) {
    //turnMotion();
  }


}
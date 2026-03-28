#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DShotRMT.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

int TUNE = 1  ; // 0 = No Tuning, 1 = Tune PIDs

float dt = 0.004; // 4ms loop time, aka 250Hz

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
int PrintCounter = 0;

float AccAngleCalibrationRoll, AccAngleCalibrationPitch;

// used for tuning, to easily send PID values to the drone without reflashing
struct PID_Parameters {
    float PRateRoll, PRatePitch, PRateYaw;
    float IRateRoll, IRatePitch, IRateYaw;
    float DRateRoll, DRatePitch, DRateYaw;
    float PAngleRoll, PAnglePitch;
    float IAngleRoll, IAnglePitch;
    float DAngleRoll, DAnglePitch;
  };

PID_Parameters pidParams;


WiFiUDP udp;
unsigned int localUdpPort = SERVER_PORT; // Port to listen on
char incomingPacket[255]; // Buffer for incomming packets



RF24 radio(25, 26);   // def pins for radio
const byte address[6] = "00001";  // set radio freq adress

float InputRoll, InputThrottle, InputPitch, InputYaw;
// Cascade Control Variables
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float throttle;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Motor pins 0, 2, 4, 15

static constexpr auto MOTOR01_PIN = 32; // Front Right
static constexpr auto MOTOR02_PIN = 27; // Rear Right
static constexpr auto MOTOR03_PIN = 33; // Rear Left
static constexpr auto MOTOR04_PIN = 13; // Front Left
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;
static constexpr auto IS_BIDIRECTIONAL = false;
static constexpr auto MOTOR01_MAGNET_COUNT = 12;

DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor02(MOTOR02_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor03(MOTOR03_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor04(MOTOR04_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);


// Gyro related functions
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+dt*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + dt * dt * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}


void gyro_signals() {

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 14, true);

  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();

  Wire.read(); Wire.read(); // skip temperature

  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 180 / PI;
  AnglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 180 / PI;
}


// Radio Related Functions/structs
struct Data_Package {
  float roll;
  float pitch;
  float yawRate;
  float throttle;
  byte  button;
};

Data_Package incomingData;


// PID related functions
class PID {
  private:
    float Kp, Ki, Kd;
    float maxOutput;
    float prevError = 0;
    float integral = 0;
    float dt; // Loop time in seconds (0.004) aka 250 Hz

  public:
    PID(float p, float i, float d, float max, float loopTime) {
      Kp = p; Ki = i; Kd = d;
      maxOutput = max;
      dt = loopTime;
    }

    void reset(){
      integral = 0;
      prevError = 0;
    }

    void updateGains(float p, float i, float d) {
      Kp = p; Ki = i; Kd = d;
    }

    float compute(float error) {
      // 1. Proportional
      float P = Kp * error;

      // 2. Integral (Trapezoidal Rule)
      integral += (error + prevError) * dt / 2.0f;

      // Clamp Integral (Anti-Windup)
      if (integral > maxOutput) integral = maxOutput;
      else if (integral < -maxOutput) integral = -maxOutput;

      float I = Ki * integral;

      // 3. Derivative
      float D = Kd * (error - prevError) / dt;

      // 4. Save state
      prevError = error;

      // 5. Final Output & Clamping
      float output = P + I + D;
      if (output > maxOutput) output = maxOutput;
      else if (output < -maxOutput) output = -maxOutput;

      return output;
    }
};

// Global PID Instances
PID PIDRateRoll(2.35, 0.05, 0.08, 800, dt);
PID PIDRatePitch(2.35, 0.05, 0.08, 800, dt);
PID PIDRateYaw(2.0, 0.05, 0.03, 800, dt);
PID PIDAngleRoll(2.0, 0, 0, 400, dt);
PID PIDAnglePitch(2.0, 0, 0, 400, dt);

void setup() {
  Serial.begin(115200); // Increased speed to match your test script
  delay(1000); // Give serial time to initialize

  Serial.println("\n\n=== ESP32 DRONE STARTING ===");
  Serial.print("TUNE mode: ");
  Serial.println(TUNE);

  // 1. Initialize I2C FIRST (before WiFi to avoid conflicts)
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // configure MPU6050 once
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Serial.println("MPU-6050 initialized");

  if (TUNE == 1) {
    Serial.println("Starting WiFi configuration...");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);

    // Use DHCP for automatic IP assignment
    Serial.println("Connecting to WiFi with DHCP...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✓ WiFi connected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      // Initialize UDP
      udp.begin(localUdpPort);
      Serial.print("UDP listening on port: ");
      Serial.println(localUdpPort);
    } else {
      Serial.println("✗ WiFi connection FAILED!");
      Serial.print("Status code: ");
      Serial.println(WiFi.status());
    }
  } else {
    Serial.println("TUNE mode disabled - WiFi not starting");
  }

  delay(1000); // Short delay before starting sensors and motors

  // Initialize PID parameters
  pidParams.PRateRoll = 2.35;
  pidParams.PRatePitch = 2.35;
  pidParams.PRateYaw = 3.5;
  pidParams.IRateRoll = 0.5;
  pidParams.IRatePitch = 0.5;
  pidParams.IRateYaw = 0.0;
  pidParams.DRateRoll = 0.04;
  pidParams.DRatePitch = 0.04;
  pidParams.DRateYaw = 0.02;
  pidParams.PAngleRoll = 1.5;
  pidParams.PAnglePitch = 1.5;
  pidParams.IAngleRoll = 0;
  pidParams.IAnglePitch = 0;
  pidParams.DAngleRoll = 0;
  pidParams.DAnglePitch = 0;

    // Update PID controllers with initial values from pidParams
  PIDRateRoll.updateGains(pidParams.PRateRoll, pidParams.IRateRoll, pidParams.DRateRoll);
  PIDRatePitch.updateGains(pidParams.PRatePitch, pidParams.IRatePitch, pidParams.DRatePitch);
  PIDRateYaw.updateGains(pidParams.PRateYaw, pidParams.IRateYaw, pidParams.DRateYaw);
  PIDAngleRoll.updateGains(pidParams.PAngleRoll, pidParams.IAngleRoll, pidParams.DAngleRoll);
  PIDAnglePitch.updateGains(pidParams.PAnglePitch, pidParams.IAnglePitch, pidParams.DAnglePitch);

  // 1. Explicitly define SPI pins for the NRF24L01
  // SCK=18, MISO=19, MOSI=23, SS/CSN=26
  SPI.begin(18, 19, 23, 26);

  // 2. Initialize Radio
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1); // Halt if radio is broken
  }
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);             // Must match your Transmitter!
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  Serial.println("Radio initialized OK!");

  // 3. Initialize Motors
  motor01.begin();
  motor02.begin();
  motor03.begin();
  motor04.begin();

  // 4. Calibration (MPU already initialized above)
  Serial.println("Calibrating Gyro... Do not move!");
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

  // Calibrate Accelerometer Angles
  for (int i=0; i<2000; i++) {
    gyro_signals();
    AccAngleCalibrationRoll += AngleRoll;
    AccAngleCalibrationPitch += AnglePitch;
    delay(1);
  }
  AccAngleCalibrationRoll /= 2000;
  AccAngleCalibrationPitch /= 2000;


  // 6. Arm ESCs (Send 0 throttle for 3 seconds)
  Serial.println("Arming ESCs...");
  for (int i = 0; i < 750; i++) {
    motor01.sendThrottle(0);
    motor02.sendThrottle(0);
    motor03.sendThrottle(0);
    motor04.sendThrottle(0);
    delay(4);
  }
  Serial.println("System Ready!");

  LoopTimer=micros();
}


void loop() {

  // Only check for UDP packets every 50 loops (5Hz instead of 250Hz)
  static int udpCheckCounter = 0;
  if (TUNE == 1) {
    udpCheckCounter++;
    if (udpCheckCounter >= 50) {  // Check every 200ms (5Hz)
      udpCheckCounter = 0;

      int packetSize = udp.parsePacket();
      if (packetSize > 0 && packetSize == sizeof(PID_Parameters)) {
        // Read the incoming packet into the PID_Parameters struct
        udp.read((char*)&pidParams, sizeof(PID_Parameters));

        // Update all PID controllers with new values
        PIDRateRoll.updateGains(pidParams.PRateRoll, pidParams.IRateRoll, pidParams.DRateRoll);
        PIDRatePitch.updateGains(pidParams.PRatePitch, pidParams.IRatePitch, pidParams.DRatePitch);
        PIDRateYaw.updateGains(pidParams.PRateYaw, pidParams.IRateYaw, pidParams.DRateYaw);
        PIDAngleRoll.updateGains(pidParams.PAngleRoll, pidParams.IAngleRoll, pidParams.DAngleRoll);
        PIDAnglePitch.updateGains(pidParams.PAnglePitch, pidParams.IAnglePitch, pidParams.DAnglePitch);

        Serial.println("✓ UDP packet received - PID updated!");
      } else if (packetSize > 0) {
        Serial.print("✗ Wrong packet size: ");
        Serial.print(packetSize);
        Serial.print(" (expected ");
        Serial.print(sizeof(PID_Parameters));
        Serial.println(")");
        udp.flush(); // Discard wrong-sized packet
      }
    }
  }

  if (radio.available()) {
    // Read the data and put it into our struct
    radio.read(&incomingData, sizeof(Data_Package));

    if (incomingData.throttle == 0) {
      throttle = 0;  // Keep zero as zero (disarmed)
    } else {
      throttle = map(incomingData.throttle, 0, 100, 48, 2047);
    }
  }

  // Sensor companastion and reading
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Subtract the calibration offset
  AngleRoll -= AccAngleCalibrationRoll;
  AnglePitch -= AccAngleCalibrationPitch;

  // 1D Kalman Filter for roll and pitch
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);


  // ONLY run PID when armed (throttle > 48)
  if (throttle > 48) {
    // Angles
    DesiredAngleRoll = incomingData.roll;
    DesiredAnglePitch = incomingData.pitch;
    DesiredRateYaw = incomingData.yawRate;

    // 2. Outer Loop (Angle PID)
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    DesiredRateRoll  = PIDAngleRoll.compute(ErrorAngleRoll);
    DesiredRatePitch = PIDAnglePitch.compute(ErrorAnglePitch);

    // 3. Inner Loop (Rate PID)
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw + RateYaw; // Needs + because of how yaw is measured (fixes stability issues)

    InputRoll = PIDRateRoll.compute(ErrorRateRoll);
    InputPitch = PIDRatePitch.compute(ErrorRatePitch);
    InputYaw = PIDRateYaw.compute(ErrorRateYaw);
  } else {
    // Disarmed - zero all PID outputs and reset integrators
    InputRoll = 0;
    InputPitch = 0;
    InputYaw = 0;

    PIDRateRoll.reset();
    PIDRatePitch.reset();
    PIDRateYaw.reset();
    PIDAngleRoll.reset();
    PIDAnglePitch.reset();
  }

  // setup motor stuff
  // Safety first

  // 4. Motor Mixing
  if (throttle > 48) { // Only spin if throttle is active
    // Quad X Mixing
    // M1(FR), M2(RR), M3(RL), M4(FL)
    MotorInput1 = throttle - InputRoll - InputPitch + InputYaw; // Front Right (CCW)
    MotorInput2 = throttle - InputRoll + InputPitch - InputYaw; // Rear Right (CW)
    MotorInput3 = throttle + InputRoll + InputPitch + InputYaw; // Rear Left (CCW)
    MotorInput4 = throttle + InputRoll - InputPitch - InputYaw; // Front Left (CW)

    // DShot CLAMPING:
    MotorInput1 = constrain(MotorInput1, 48, 2047);
    MotorInput2 = constrain(MotorInput2, 48, 2047);
    MotorInput3 = constrain(MotorInput3, 48, 2047);
    MotorInput4 = constrain(MotorInput4, 48, 2047);
  } else {
    // Kill motors if throttle low (PIDs already reset above)
    MotorInput1 = 0;
    MotorInput2 = 0;
    MotorInput3 = 0;
    MotorInput4 = 0;
  }

  // Sending the final Dshot signals to each motor
  motor01.sendThrottle(MotorInput1);
  motor02.sendThrottle(MotorInput2);
  motor03.sendThrottle(MotorInput3);
  motor04.sendThrottle(MotorInput4);

  // --- Print every 25th loop (100ms at 250Hz) ---
  PrintCounter++;
  if (PrintCounter >= 25) {
    // All on ONE line for easy logging/plotting
    Serial.print("Thr:"); Serial.print((int)throttle);
    Serial.print(" | InR:"); Serial.print((int)InputRoll);
    Serial.print(" InP:"); Serial.print((int)InputPitch);
    Serial.print(" InY:"); Serial.print((int)InputYaw); 
    Serial.print(" | M1:"); Serial.print((int)MotorInput1);
    Serial.print(" M2:"); Serial.print((int)MotorInput2);
    Serial.print(" M3:"); Serial.print((int)MotorInput3);
    Serial.print(" M4:"); Serial.print((int)MotorInput4);
    Serial.print(" | Gy-R:"); Serial.print(RateRoll, 1);
    Serial.print(" Gy-P:"); Serial.print(RatePitch, 1);
    Serial.print(" Gy-Y:"); Serial.print(RateYaw, 1);
    Serial.print(" | Ka-R:"); Serial.print(KalmanAngleRoll, 1);
    Serial.print(" Ka-P:"); Serial.println(KalmanAnglePitch, 1);

    // Print PID parameters on a new line
    Serial.print("PID | RR-P:"); Serial.print(pidParams.PRateRoll, 2);
    Serial.print(" I:"); Serial.print(pidParams.IRateRoll, 2);
    Serial.print(" D:"); Serial.print(pidParams.DRateRoll, 3);
    Serial.print(" | RP-P:"); Serial.print(pidParams.PRatePitch, 2);
    Serial.print(" I:"); Serial.print(pidParams.IRatePitch, 2);
    Serial.print(" D:"); Serial.print(pidParams.DRatePitch, 3);
    Serial.print(" | RY-P:"); Serial.print(pidParams.PRateYaw, 2);
    Serial.print(" I:"); Serial.print(pidParams.IRateYaw, 2);
    Serial.print(" D:"); Serial.print(pidParams.DRateYaw, 3);
    Serial.print(" | AR-P:"); Serial.print(pidParams.PAngleRoll, 2);
    Serial.print(" I:"); Serial.print(pidParams.IAngleRoll, 2);
    Serial.print(" D:"); Serial.print(pidParams.DAngleRoll, 3);
    Serial.print(" | AP-P:"); Serial.print(pidParams.PAnglePitch, 2);
    Serial.print(" I:"); Serial.print(pidParams.IAnglePitch, 2);
    Serial.print(" D:"); Serial.println(pidParams.DAnglePitch, 3);

    PrintCounter = 0; // Reset the counter
  }

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

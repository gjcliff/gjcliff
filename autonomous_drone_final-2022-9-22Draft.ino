//by Graham Clifford
//test
//initializing libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "MPU6050.h"
#include <SPI.h>
#include "RF24.h"
#include "ArduPID.h"

//PID Variables
ArduPID pid_y;
ArduPID pid_p;
ArduPID pid_r;
ArduPID pid_az;
double p = 1;
double i = 1;
double d = 0;

//receiver variables
RF24 radio(A0, 10); // radio(CE_pin,CSN_pin)
uint8_t address[][6] = {"1Node", "2Node"}; // Let these addresses be used for the pair
char data[33] = {}; //32 bytes is max transmission size, +1 bite for NULL byte at end of string.
char ack[33] = {};

//motor variables
const uint8_t motor[] = {3, 5, 6, 9};
uint8_t ground[] = {A1, 4, 7, 8};
double thrust_mod = 0;
uint16_t thrust[] = {0, 0, 0, 0};

//Gyroscope and Accelerometer Variables
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL
#define INTERRUPT_PIN 2

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double _ypr[3] = {0.0, 0.0, 0.0};
double _accel[3] = {0.0, 0.0, 0.0}; // [ax,ay,az] acceleration in x/y/z container
double ypr_setpoint[3] = {0.0, 0.0, 0.0}; //Define set points so we can do turns and stuff.
double accel_setpoint[3] = {0.0, 0.0, 0.0}; //Define set points so we can move in different directions.

// ================================================================
// ===                   INTERRUPT DETECTION                    ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// join I2C bus (I2Cdev library doesn't do this automatically

// ================================================================
// ===                  Radio Initialization                 ===
// ================================================================
void radioInit() {
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }
  Serial.println(F("radio initialized"));
  radio.enableAckPayload(); //Allow ack(nowledgement) payloads. This will let us send data back to transmitter without manually changing the radio modes on both Arduinos.
  radio.enableDynamicPayloads(); //Need for sending ack payloads on pipes other than 0 and 1. This enables it on all pipes.
  radio.setRetries(5, 15);
  radio.openWritingPipe(address[1]);     // always uses pipe 0
  radio.openReadingPipe(1, address[0]); // using pipe 1

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the number of bytes we need to transmit a float

  // additional setup specific to the node's role
  radio.startListening(); // put radio in RX mode
}

// ================================================================
// ===                Motor Driver Initialization               ===
// ================================================================

void motorDriverInit() {
  Serial.println("Initializing motor pins...");
  for (int i = 0; i < 4; i++) {
    pinMode(motor[i], OUTPUT);
    analogWrite(motor[i], 0);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(ground[i], OUTPUT);
    digitalWrite(ground[i], LOW);
  }
}

// ================================================================
// ===        Gyroscope and Accelerometer Initialization        ===
// ================================================================
void gyroAccelerometerInit() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#endif

  //transmitString("Initializing I2C devices...");
  mpu.initialize(); //initialize the motion processing unit

  pinMode(INTERRUPT_PIN, INPUT); //set pin 2 as the interrupt pin. Is the MPU6050 sending its own interrupts?

  //verify connection
  //transmitString("Testing device connections...");
  mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed");

  // load and configure the DMP
  //transmitString("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    transmitString("MPU Calibrated");
    // turn on the DMP, now that it's ready
    //transmitString("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //transmitString("Enabling interrupt detection....");
    digitalPinToInterrupt(INTERRUPT_PIN);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    transmitString("$");

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    transmitString("DMP Initialization failed (code " + String(devStatus) + ")");
    Serial.print("error: "); Serial.println(devStatus);
  }
}

void transmitString(String s) {
  uint8_t len = s.length() + 1;
  char data[len];
  s.toCharArray(data, len);
  if (radio.write(&data, sizeof(data))) {
  }
}

void pidInit(int j, ArduPID pid, int k) {
  if (k == 0) {
    pid.begin(&_ypr[j], &thrust_mod, &ypr_setpoint[j], p, i, d);
    pid.setOutputLimits(0, 255);
    pid.setBias(255.0 / 2.0);
    pid.setWindUpLimits(-10, 10);
    pid.setSampleTime(10);
    pid.start();
  }
  else {
    double az = aaWorld.z;
    pid.begin(&az, &thrust_mod, &accel_setpoint[2], p, i, d);
    pid.setOutputLimits(0, 255);
    pid.setWindUpLimits(-10, 10);
    pid.setBias(255.0 / 2.0);
    pid.setSampleTime(10);
    pid.start();
  }
}

void setup() {
  Serial.begin(19200);
  radioInit();
  motorDriverInit();
  Serial.println("Waiting for transmission from controller to begin MPU6050 intitialization");  //wait to receive a tranmission from the controller before initializing the gyroscope accelerometer
  while (!radio.available()) {}
  radio.stopListening();
  gyroAccelerometerInit();
  radio.startListening();

  //pid_y.begin(&_ypr[0],&thrust_mod,&ypr_setpoint[0],p,i,d);
  pid_p.begin(&_ypr[1], &thrust_mod, &ypr_setpoint[1], p, i, d);
  pid_r.begin(&_ypr[2], &thrust_mod, &ypr_setpoint[2], p, i, d);
  pid_az.begin(&_accel[2], &thrust_mod, &accel_setpoint[2], p, i, d);

  /*pidInit(0, pid_y, 0);
    pidInit(1, pid_p, 0);
    pidInit(2, pid_r, 0);
    pidInit(2, pid_az, 1);*/
  for (int i = 0; i < 4; i ++) {
    thrust[i] += 50;
  }
}

void loop() {
  // ================================================================
  // ===               Gyroscope/Accelerometer Loop               ===
  // ================================================================
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  for (int j = 0; j < 3; j++) {
    _ypr[j] = ypr[j]* 180;
  }

  _accel[2] = aaWorld.z;

  if (radio.available()) {
    int bytes = radio.getPayloadSize();
    radio.read(&data, bytes);
    radio.writeAckPayload(0, &ack, bytes);
  }

  // ================================================================
  // ===                        Flight Loop                       ===
  // ================================================================
  //Leveling Logic
  pid_y.compute();
  thrust[0] -= thrust_mod;
  thrust[1] += thrust_mod;
  thrust[2] += thrust_mod;
  thrust[3] -= thrust_mod;

  pid_p.compute();

  thrust[0] -= thrust_mod;
  thrust[1] -= thrust_mod;
  thrust[2] += thrust_mod;
  thrust[3] += thrust_mod;

  pid_r.compute();

  thrust[0] -= thrust_mod;
  thrust[1] += thrust_mod;
  thrust[2] -= thrust_mod;
  thrust[3] += thrust_mod;

  pid_az.compute();
  thrust[0] += thrust_mod;
  thrust[1] += thrust_mod;
  thrust[2] += thrust_mod;
  thrust[3] += thrust_mod;

  for (int j = 0; j < 4; j++) {
    if (thrust[j] > 255) {
      thrust[j] = 200;
    }
  }

  Serial.print("thrust mod: "); Serial.println(thrust_mod);
  Serial.print("ypr: "); Serial.print(_ypr[0]); Serial.print("\t"); Serial.print(_ypr[1]); Serial.print("\t"); Serial.print(_ypr[2]); Serial.print("\t");
  for (int j = 0; j < 4; j++) {
    analogWrite(motor[j], thrust[j]);
    Serial.print("thrust: "); Serial.println(thrust[0]);
  }
}

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
// #include <SD.h>

#define MICRO_SD_CS 10
#define SOFTWARE_SERIAL_RX 4 // need to be connected to TX of GPS module
#define SOFTWARE_SERIAL_TX 3 // need to be connected to RX of GPS module
#define SERVO_PWM 6

#define SERVO_ORIGINAL_POS 0
#define SERVO_TRIGGERED_POS 90

// Arduino Pin Connection Description

/* I2C: A5(SCL), A4(SDA)
  <I2C Device>
  MPU6050
  BMP280
 */
/* SPI: 10(CS), 11(MOSI), 12(MISO), 13(SCK)
  <SPI Device>
  microSD reader
 */
/* Software Serial (UART by SW): 4(RX), 3(TX)
  <UART Device>
  Neo-6m (GPS receiver)
*/
/* PWM
  <PWM Device>
  Servo motor (pin 6)
*/

typedef struct sensorValues { // define struct to save the newest collected data from sensors
  float roll_deg; // deg
  float pitch_deg; // deg
  float ambient_pressure_hpa; // atm
  float longitude_deg; // deg
  float latitude_deg; // deg
  bool servo_triggered; // 1 if triggered, 0 otherwise.
} State;

float last_lon_deg;
float last_lat_deg;
bool servo_triggered;

// IMU variables
float RateCalibrationRoll, RateCalibrationPitch;
uint32_t prevLoopTimer, currLoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(float* RateRoll, float* RatePitch, float* AngleRoll, float* AnglePitch) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  *RateRoll=(float)GyroX/65.5 - RateCalibrationRoll;
  *RatePitch=(float)GyroY/65.5 - RateCalibrationPitch;
  float AccX=(float)AccXLSB/4096;
  float AccY=(float)AccYLSB/4096;
  float AccZ=(float)AccZLSB/4096;
  *AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  *AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

// BMP object
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// GPS object
TinyGPS gps;
// SoftwareSerial ss(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);

// Servo object
Servo myservo;  // create Servo object to control a servo

// SD card file
// File myFile;

// timer variables
uint32_t prevTime5HzCounter;
uint32_t currTime5HzCounter;
uint32_t prevTime1HzCounter;
uint32_t currTime1HzCounter;

void setup() {
  // Serial.begin(115200);
  //Serial.println("test");
  testSerialCommunication();
  initializeIMU(&RateCalibrationRoll, &RateCalibrationPitch);
  initializeBMP280();
  initializeGPS();
  initializeServo();
  //initializeSD();
  prevTime5HzCounter = millis();
  prevTime1HzCounter = millis();
}

void loop() {
  // always read MPU for attitude estimation
  updateIMUstate(); // calling this function automatically update attitude in 400Hz freq.
  // always check servo trigger attitude has acheived
  if (isAttitudeConditionSatisfied()) { // Servo trigger to 90 deg at pitch > 50 deg
    myservo.write(90);
    servo_triggered = true;
  } else {
    myservo.write(0);
    servo_triggered = false;
  }

  uint32_t currTime = millis();
  currTime5HzCounter = currTime;
  currTime1HzCounter = currTime;

  if (currTime5HzCounter - prevTime5HzCounter > 100) { // if 0.2 sec has passed since the last reading
    prevTime5HzCounter = currTime5HzCounter; // update timer

    // State Variable
    State myRocketState; // define new State variable

    // Euler angle (RPY), 5Hz
    // get newest RPY attitude in degeree.
    myRocketState.pitch_deg = KalmanAnglePitch; 
    myRocketState.roll_deg = KalmanAngleRoll;

    // BMP280, pressure, 5Hz
    sensors_event_t pressure_event;
    bmp_pressure->getEvent(&pressure_event);
    myRocketState.ambient_pressure_hpa = pressure_event.pressure; // hpa

    // Send collected data to computer by Serial
    myRocketState.servo_triggered = servo_triggered;
    myRocketState.latitude_deg = last_lat_deg;
    myRocketState.longitude_deg = last_lon_deg;

    sendToPC(&myRocketState);

    // Save collected data to 'log.dat' file
    //saveToSD(&myRocketState);

  }

  if (currTime1HzCounter - prevTime1HzCounter > 1000) { // if 1 sec has passed since the last reading
    prevTime1HzCounter = currTime1HzCounter;
    // GPS, lon, lat, 1Hz
    readGPS();
  }
}


// User defined function.
void testSerialCommunication() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println("PC Serial Communication Test Done!");
}

void initializeIMU(float* RateCalibrationRoll, float* RateCalibrationPitch) {
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.print("IMU Calibration");
  
  float RateCalRoll = 0; 
  float RateCalPitch = 0;

  for (int i=0; i<2000; i ++) {
    float RateRoll, RatePitch, RateYaw;
    readGyroNoCal(&RateRoll, &RatePitch);
    RateCalRoll+=RateRoll;
    RateCalPitch+=RatePitch;
    delay(1);
    if (i % 100 == 0) Serial.print(".");
  }
  Serial.println();
  RateCalRoll/=2000;
  RateCalPitch/=2000;

  *RateCalibrationRoll = RateCalRoll;
  *RateCalibrationPitch = RateCalPitch;

  prevLoopTimer=micros();
}

void updateIMUstate() {
  currLoopTimer = micros();
  if (currLoopTimer - prevLoopTimer < 4000) return;

  prevLoopTimer = currLoopTimer;
  float RateRoll, RatePitch, AngleRoll, AnglePitch;
  gyro_signals(&RateRoll, &RatePitch, &AngleRoll, &AnglePitch);

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  // Serial.print("Roll Angle [°] ");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" Pitch Angle [°] ");
  // Serial.println(KalmanAnglePitch);
}

void initializeBMP280() {
  Serial.print("Initializing BMP...");
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("BMP initialization Done!");
}

void initializeGPS() {
  Serial.print("Initializing GPS...");
  // ss.begin(9600);
  Serial.println("GPS initialization Done!");
}

void initializeServo() {
  Serial.print("Initializing Servo...");
  myservo.attach(SERVO_PWM);  // attaches the servo on pin 9 to the Servo object
  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  Serial.println("Servo initialization Done!");
}

// void initializeSD() {
//   Serial.print("Initializing SD card...");

//   if (!SD.begin(MICRO_SD_CS)) {
//     Serial.println("initialization failed!");
//     while (1);
//   }
//   myFile = SD.open("log.data", FILE_WRITE);
//   Serial.println("SD initialization Done.");
// }

void readGPS() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat, flon;
  unsigned long age;
  int satNum;

  if (Serial.available()) {
      char c = Serial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    flat = (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    flon = (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    satNum = (gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  }
  
  gps.stats(&chars, &sentences, &failed);
  // Serial.println(chars);
  if (chars == 0)
    // Serial.println("** No characters received from GPS: check wiring **");
    flat = 0.0;
    flon = 0.0;

  last_lat_deg = flat;
  last_lon_deg = flon;
}

void sendToPC(State* pData) {
  Serial.print((float)currTime5HzCounter / 1000.0);
  Serial.print(" sec, ");

  Serial.print(" RPY: ");
  Serial.print(pData->roll_deg);
  Serial.print(" deg, ");
  Serial.print(pData->pitch_deg);
  Serial.print(" deg, ");

  Serial.print(" GPS: ");
  Serial.print(pData->latitude_deg);
  Serial.print(" deg, ");
  Serial.print(pData->longitude_deg);
  Serial.print(" deg, ");

  Serial.print(" Atm: ");
  Serial.print(pData->ambient_pressure_hpa);
  Serial.print(" hpa, ");

  Serial.print("ServoPos: ");
  Serial.print(pData->servo_triggered);
  
  Serial.println();
}

// void saveToSD(State* pData) {
//   myFile.write((const uint8_t*)pData, sizeof(State));
// }

bool isAttitudeConditionSatisfied() {
  if (abs(KalmanAnglePitch) >= 50) return true;
  else return false;
}

void readGyroNoCal(float* gyrox, float* gyroy) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,4);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  *gyrox = (float)GyroX/65.5;
  *gyroy = (float)GyroY/65.5;
}
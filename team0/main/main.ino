#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>

#define MICRO_SD_CS 10
#define SOFTWARE_SERIAL_RX 4 // need to be connected to TX of GPS module
#define SOFTWARE_SERIAL_TX 3 // need to be connected to RX of GPS module
#define SERVO_PWM 6

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
  float yaw_deg; // deg
  float ambient_pressure_atm; // atm
  float longitude_deg; // deg
  float latitude_deg; // deg
  bool servo_triggered; // 1 if triggered, 0 otherwise.
} State;

// BMP object
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// GPS object
TinyGPS gps;
SoftwareSerial ss(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);

// Servo object
Servo myservo;  // create Servo object to control a servo

// SD card file
File myFile;


// timer variables
long prevTime = millis();
long currTime;


void setup() {
  /***************** Serial Initialize *******************/
  Serial.begin(115200);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println("PC Serial Communication Test Done!");
  


  /***************** MPU Initialize *******************/
  Wire.begin();


  /***************** BMP280 Initialize *******************/
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



  /***************** GPS Initialize *******************/
  ss.begin(9600);
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();

  /***************** Servo Initialize *******************/
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
  /***************** microSD Initialize *******************/
  Serial.print("Initializing SD card...");

  if (!SD.begin(MICRO_SD_CS)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

}

void loop() {
  // always read MPU for attitude estimation

  // Euler angle (RPY), 5Hz
  // currTime = millis();
  // if (currTime - prevTime > 200) { // if 0.2 sec has passed from the last RPY reading
  //   prevTime = currTime; // update prevTime for next call

  // }

  // BMP280, pressure, 5Hz

  // GPS, lon, lat, 1Hz

  // Servo trigger to 90 deg at pitch > 50 deg

  // Send collected data to computer by Serial

  // Save collected data to 'log.dat' file

}
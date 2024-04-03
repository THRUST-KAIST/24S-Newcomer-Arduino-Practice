#include <Wire.h>

#define SAMPLING_FREQ 400

typedef struct {
  uint32_t time; // microsec
  float accX; // g
  float accY; // g
  float accZ; // g
  float gyroX; // deg/s
  float gyroY; // deg/s
  float gyroZ; // deg/s
} rawAccGyro;
    

class IMU {
  private:
    // float accX, accY, accZ;
    // float gyroX, gyroY, gyroZ;
    int16_t gyroX_LSB_cal, gyroY_LSB_cal, gyroZ_LSB_cal;
    uint32_t loopTimer;
    int16_t dt_micros = 1E6/SAMPLING_FREQ;

    void getAccLSB(int16_t* accX_LSB, int16_t* accY_LSB, int16_t* accZ_LSB) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); // Registers 59: Gyroscope Measurements
      Wire.endTransmission(); 
      Wire.requestFrom(0x68,6);
      *accX_LSB = Wire.read() << 8 | Wire.read();
      *accY_LSB = Wire.read() << 8 | Wire.read();
      *accZ_LSB = Wire.read() << 8 | Wire.read();
    }

    void getGyroLSB(int16_t* gyroX_LSB, int16_t* gyroY_LSB, int16_t* gyroZ_LSB) {
      Wire.beginTransmission(0x68);
      Wire.write(0x43); // Registers 67: Gyroscope Measurements
      Wire.endTransmission();
      Wire.requestFrom(0x68,6);
      *gyroX_LSB =Wire.read()<<8 | Wire.read();
      *gyroY_LSB =Wire.read()<<8 | Wire.read();
      *gyroZ_LSB =Wire.read()<<8 | Wire.read();
    }

    void calibrateGyro() {
      // Serial.print("IMU Calibration");
      
      int64_t cal_x_gyro = 0, cal_y_gyro = 0, cal_z_gyro = 0;
      int16_t gyroX_LSB, gyroY_LSB, gyroZ_LSB;
      for (int i=0; i<3000; i ++) {
        this->getGyroLSB(&gyroX_LSB, &gyroY_LSB, &gyroZ_LSB);
        cal_x_gyro += gyroX_LSB;
        cal_y_gyro += gyroY_LSB;
        cal_z_gyro += gyroZ_LSB;
        delay(1);
        // if (i % 100 == 0) Serial.print(".");
      }
      // Serial.println();
      // get average bias
      cal_x_gyro /= 3000;
      cal_y_gyro /= 3000;
      cal_z_gyro /= 3000;
    
      this->gyroX_LSB_cal = (int16_t) cal_x_gyro;
      this->gyroY_LSB_cal = (int16_t) cal_y_gyro;
      this->gyroZ_LSB_cal = (int16_t) cal_z_gyro;
    }


  public:
    void setup() {
      // I2C begin
      Wire.setClock(400000);
      Wire.begin();
      delay(250);

      // Power Management 1
      Wire.beginTransmission(0x68); 
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission();

      // Configuration
      Wire.beginTransmission(0x68);
      Wire.write(0x1A);
      Wire.write(0x05);
      Wire.endTransmission();

      //  Accelerometer Configuration
      Wire.beginTransmission(0x68);
      Wire.write(0x1C);
      Wire.write(0x10); // LSB/g is 4096
      Wire.endTransmission();

      // Gyro Configuraton
      Wire.beginTransmission(0x68);
      Wire.write(0x1B); 
      Wire.write(0x8); // LSB/deg/s is 65.5
      Wire.endTransmission();

      this->calibrateGyro();

      this->loopTimer = micros(); // initialize loopTimer
    }

  void getAcc(float* accX, float* accY, float* accZ) {
    // output unit is 'g'
    int16_t accX_LSB, accY_LSB, accZ_LSB;
    this->getAccLSB(&accX_LSB, &accY_LSB, &accZ_LSB);

    *accX = (float) accX_LSB / 4096.0; // g
    *accY = (float) accY_LSB / 4096.0; // g
    *accZ = (float) accZ_LSB / 4096.0; // g
  }

  void getGyro(float* gyroX, float* gyroY, float* gyroZ) {
    // output unit is 'deg/s'
    int16_t gyroX_LSB, gyroY_LSB, gyroZ_LSB;
    this->getGyroLSB(&gyroX_LSB, &gyroY_LSB, &gyroZ_LSB);

    // apply calibration
    gyroX_LSB -= (this->gyroX_LSB_cal);
    gyroY_LSB -= (this->gyroY_LSB_cal);
    gyroZ_LSB -= (this->gyroZ_LSB_cal);

    *gyroX = (float) gyroX_LSB / 65.5; // deg/s
    *gyroY = (float) gyroY_LSB / 65.5; // deg/s
    *gyroZ = (float) gyroZ_LSB / 65.5; // deg/s
  }

  void getNewReading() {
    rawAccGyro data;

    uint32_t currLoopTimer = micros();
    if (currLoopTimer - this->loopTimer < this->dt_micros) { // 250 Hz timer
      return; 
    }
    this->getAcc(&(data.accX), &(data.accY), &(data.accZ));
    this->getGyro(&(data.gyroX), &(data.gyroY), &(data.gyroZ));
    data.time = currLoopTimer; // record current time
    
    Serial.write((const uint8_t *)&data, sizeof(data)); // Write raw data on the serial bus.

    this->loopTimer = currLoopTimer;
  }
  
};

IMU myIMU; // define my IMU object

void setup() {
  Serial.begin(115200);
  myIMU.setup(); // initialize IMU
}

void loop() {
  myIMU.getNewReading(); // print out acc and gyro raw values, 250 Hz
}
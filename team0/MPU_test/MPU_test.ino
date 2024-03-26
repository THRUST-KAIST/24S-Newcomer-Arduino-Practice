//Include I2C library and declare variables
#include <Wire.h>

class MPU6050_Attitude {
  private:
    int gyro_x_cal;
    int gyro_y_cal;
    int gyro_z_cal;

    long attitudeLoopTimer = micros();

    float tau = 0.98; // complementary filter weight
    float LPF = 1; 

    float freq = 250; // Hz
    float dt = 1/250; // sec

    float p; // rad/s
    float q; // rad/s
    float r; // rad/s
    float a_xb; // g, imu reading
    float a_yb; // g, imu reading
    float a_zb; // g, imu reading

    void read_gyro_acc() {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 14);

      // Read data --> Temperature falls between acc and gyro registers
      long acc_x = Wire.read() << 8 | Wire.read();
      long acc_y = Wire.read() << 8 | Wire.read();
      long acc_z = Wire.read() << 8 | Wire.read();
      long temperature = Wire.read() <<8 | Wire.read();
      long gyro_x = Wire.read()<<8 | Wire.read();
      long gyro_y = Wire.read()<<8 | Wire.read();
      long gyro_z = Wire.read()<<8 | Wire.read();

      gyro_x -= this->gyro_x_cal;
      gyro_y -= this->gyro_y_cal;
      gyro_z -= this->gyro_z_cal;

      this->p = ((float)gyro_x / 131.0 * DEG_TO_RAD); // rad/s
      this->q = ((float)gyro_y / 131.0 * DEG_TO_RAD); // rad/s
      this->r = ((float)gyro_z / 131.0 * DEG_TO_RAD); // rad/s

      this->a_xb = ((float)acc_x / 8192); // g
      this->a_yb = ((float)acc_y / 8192); // g
      this->a_zb = ((float)acc_z / 8192); // g
    }

    void read_gyro_raw(long* gyro_x, long* gyro_y, long* gyro_z) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 14);

      // Read data --> Temperature falls between acc and gyro registers
      long acc_x = Wire.read() << 8 | Wire.read();
      long acc_y = Wire.read() << 8 | Wire.read();
      long acc_z = Wire.read() << 8 | Wire.read();
      long temperature = Wire.read() <<8 | Wire.read();
      *gyro_x = Wire.read()<<8 | Wire.read();
      *gyro_y = Wire.read()<<8 | Wire.read();
      *gyro_z = Wire.read()<<8 | Wire.read();
    }
  
  public:
    float roll = 0; // rad
    float pitch = 0; // rad
    float yaw = 0; // rad


    void setup() {
      //Activate the MPU-6050
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission();

      // Configure the accelerometer
      // Wire.write(0x__);
      // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
      Wire.beginTransmission(0x68);
      Wire.write(0x1C);
      Wire.write(0x08);
      Wire.endTransmission();

      // Configure the gyro
      // Wire.write(0x__);
      // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
      Wire.beginTransmission(0x68);
      Wire.write(0x1B);
      Wire.write(0x00); // 250 deg/s
      Wire.endTransmission();
    }

    void update_attitude() {
      long new_attitudeLoopTimer = micros();
      if (new_attitudeLoopTimer - this->attitudeLoopTimer < 4000) {
        // no need to update
        return;
      }
      this->attitudeLoopTimer = new_attitudeLoopTimer; // update the time last attitude was updated
      this->read_gyro_acc();
      float p = this->p;
      float q = this->q;
      float r = this->r;

      float roll = this->roll;
      float pitch = this->pitch;
      float yaw = this->yaw;

      float sPhi = sin(roll);
      float cPhi = cos(roll);
      float sTheta = sin(pitch);
      float cTheta = cos(pitch);

      float phi_dot = p + (sPhi*q + cPhi*r)*sTheta/cTheta;
      float theta_dot = cPhi*q - sPhi*r;
      float psi_dot = (sPhi*q + cPhi*r)/cTheta;

      Serial.println(psi_dot);

      float dt = this->dt; // time step, sec

      // update RPY using Euler Method + complementary filter
      float theta_acc = asin(-(this->a_xb));
      float phi_acc = atan2(this->a_yb, this->a_zb);
      float tau = this->tau;
      this->roll = (roll + phi_dot * dt)*tau + phi_acc*(1-tau);
      this->pitch = (pitch + theta_dot * dt)*tau + theta_acc*(1-tau);
      this->yaw = yaw + psi_dot * dt;

      // Data out serial monitor
      Serial.print(this->roll * RAD_TO_DEG);   Serial.print(",");
      Serial.print(this->pitch * RAD_TO_DEG);  Serial.print(",");
      Serial.println(this->yaw * RAD_TO_DEG);
    }

    void get_RPY_DEG(float* roll, float* pitch, float* yaw) {
      *roll = (this->roll) * RAD_TO_DEG;
      *pitch = (this->pitch) * RAD_TO_DEG;
      *yaw = (this->yaw) * RAD_TO_DEG;
    }

    void calibrate_gyro() {
      Serial.println("Calibrating gyro, place on level surface and do not move.");
      // Take 3000 readings for each coordinate and then find average offset
      long gyro_x_cal = 0;
      long gyro_y_cal = 0;
      long gyro_z_cal = 0;
      long gyro_x, gyro_y, gyro_z;
      for (int cal_int = 0; cal_int < 3000; cal_int ++){
        if(cal_int % 200 == 0)Serial.print(".");
        read_gyro_raw(&gyro_x, &gyro_y, &gyro_z);
        gyro_x_cal += gyro_x;
        gyro_y_cal += gyro_y;
        gyro_z_cal += gyro_z;
        delay(3);
      }

      // Average the values
      gyro_x_cal /= 3000;
      gyro_y_cal /= 3000;
      gyro_z_cal /= 3000;

      this->gyro_x_cal = gyro_x_cal;
      this->gyro_y_cal = gyro_y_cal;
      this->gyro_z_cal = gyro_z_cal;
    }

};

MPU6050_Attitude myIMU;

void setup() {
  // Start
  Wire.begin();
  Serial.begin(115200);

  // Setup the registers of the MPU-6050 and start up
  myIMU.setup();

  // Calibration
  myIMU.calibrate_gyro();

  // Display headers
  Serial.print("\nNote 1: Yaw is not filtered and will drift!\n");
  Serial.print("\nNote 2: Make sure sampling frequency is ~250 Hz\n");
  Serial.print("Sampling Frequency (Hz)\t\t");
  Serial.print("Roll (deg)\t\t");
  Serial.print("Pitch (deg)\t\t");
  Serial.print("Yaw (deg)\t\t\n");
  delay(2000);
}


void loop() {
  myIMU.update_attitude();
}
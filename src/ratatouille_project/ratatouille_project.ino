#include <Wire.h>
#include <Servo.h>

/*
 * Definition of MPU-6050 Address
 * These registeres can be found on MPU-6050 Register Map
 */
const int MPU_ADDR_1 = 0x68;   // First MPU-6050 Address 
const int MPU_ADDR_2 = 0x69;   // Second MPU-6050 Address

const int WHO_AM_I = 0x75;     // Device identification register
const int PWR_MGMT_1 = 0x6B;   // Power management register
const int GYRO_CONFIG = 0x1B;  // Gyroscope config register
const int ACCEL_CONFIG = 0x1C; // Acelerometer config register
const int ACCEL_XOUT = 0x3B;   // X axis read register

// I2C SDA and SCL Pin definition
const int sda_pin = D5;
const int scl_pin = D6;

// Variables to store sensor raw data
int16_t aX, aY, aZ, tmp, gX, gY, gZ;

Servo servo;

void initMPU(int mpu_addr) {
  Wire.beginTransmission(mpu_addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // wake up
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(mpu_addr);
  Wire.write(GYRO_CONFIG);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(mpu_addr);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(10);
}

/*
 * This function reads the data from a register
 */
uint8_t readRegMPU(int mpu_addr, uint8_t reg) {
  uint8_t data;
  Wire.beginTransmission(mpu_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 1);
  data = Wire.read();
  return data;
}

/*
 * This function searches for a sensor on 0x68 address
 */
void findMPU(int mpu_addr) {
  Wire.beginTransmission(mpu_addr);
  int data = Wire.endTransmission(true);

  if (data == 0) {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(mpu_addr, HEX);
  } else {
    Serial.println("Dispositivo não encontrado!");
  }
}

/*
 * This function verifies if the sensor is active
 */
void checkMPU(int mpu_addr) {
  findMPU(mpu_addr);

  int data = readRegMPU(mpu_addr, WHO_AM_I);  // Register 117 – Who Am I - 0x75

  if (data == 104) {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    data = readRegMPU(mpu_addr, PWR_MGMT_1);  // Register 107 – Power Management 1-0x6B

    if (data == 64) {
      Serial.println("MPU6050 em modo SLEEP! (64)");
    } else {
      Serial.println("MPU6050 em modo ACTIVE!");
    }
  } else {
    Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
  }
}

/*
 * This function initializes the Servo
 */
void initServo() {
  servo.attach(2); // D4
  servo.write(0);
  delay(1000);
  servo.write(180);
  delay(1000);
  servo.write(0);
}

/**
  * This function reads sensor raw data
  * In total, 14 bytes are read, being:
  * - 2 Bytes for each axis for Acelerometer (6 in total)
  * - 2 Bytes for each axis for Gyroscope    (6 in total)
  * - 2 Bytes for temperature
  *
  * returns the X roll Angle in degrees
  */
int readRawMPU(int mpu_addr) {
  Wire.beginTransmission(mpu_addr);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 14);

  aX = Wire.read() << 8;  // Read most significant byte first
  aX |= Wire.read();      // OR with less significant byte
  aY = Wire.read() << 8;
  aY |= Wire.read();
  aZ = Wire.read() << 8;
  aZ |= Wire.read();

  tmp = Wire.read() << 8;
  tmp |= Wire.read();

  gX = Wire.read() << 8;
  gX |= Wire.read();
  gY = Wire.read() << 8;
  gY |= Wire.read();
  gZ = Wire.read() << 8;
  gZ |= Wire.read();

  // Converts the aceleration to 'g'
  float ax_g = aX / 16384.0;
  float ay_g = aY / 16384.0;
  float az_g = aZ / 16384.0;

  // Compute the degrees
  // float pitch = atan2(ay_g, az_g) * 180.0 / PI;
  float roll = atan2(ax_g, az_g) * 180.0 / PI;

  return (int) roll;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("Starting MPU-6050 Setup");

  Serial.println("Initializing I2C");
  Wire.begin(sda_pin, scl_pin);

  Serial.println("Setup MPU");
  initMPU(MPU_ADDR_1);
  initMPU(MPU_ADDR_2);

  checkMPU(MPU_ADDR_1);
  checkMPU(MPU_ADDR_2);

  //initServo();

  Serial.println("Setup finished. Starting Loop...");
  delay(2000);
}

void loop() {
  int roll1 = readRawMPU(MPU_ADDR_1);
  int roll2 = readRawMPU(MPU_ADDR_2);

  // Valule, Min, Max, Target Min, Target Max
  int angle1 = map(roll1, -90, 90, 0, 180);
  int angle2 = map(roll2, -90, 90, 0, 180);

  Serial.print("Pos 1: "); Serial.println(roll1);
  Serial.print("Angle: "); Serial.println(angle1);
  Serial.print("Pos 2: "); Serial.println(roll2);
  Serial.print("Angle: "); Serial.println(angle2);
  Serial.println("=====================");

  //servo.write(angle);

  delay(200);
}
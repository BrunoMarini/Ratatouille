#include <Wire.h>

/*
 * Definições de alguns endereços mais comuns do MPU6050
 * os registros podem ser facilmente encontrados no mapa de registros do MPU6050
 */
const int MPU_ADDR = 0x68;      // MPU-6050 Address
const int WHO_AM_I = 0x75;      // Device identification register
const int PWR_MGMT_1 = 0x6B;    // Power management register
const int GYRO_CONFIG = 0x1B;   // Gyroscope config register
const int ACCEL_CONFIG = 0x1C;  // Acelerometer config register
const int ACCEL_XOUT = 0x3B;    // X axis read register

const int sda_pin = D5;  // definição do pino I2C SDA
const int scl_pin = D6;  // definição do pino I2C SCL

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t aX, aY, aZ, tmp, gX, gY, gZ;

/*
 * This function setups the I2C communication with the defined pins
 */
void initI2C() {
  Serial.println("initI2C()");
  Wire.begin(sda_pin, scl_pin);
}

/*
 * This function writes a value on a register
 */
void writeRegMPU(int reg, int val) {
  Wire.beginTransmission(MPU_ADDR);  
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

/*
 * This function reads the data from a register
 */
uint8_t readRegMPU(uint8_t reg) {
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  data = Wire.read();
  return data;
}

/*
 * This function searches for a sensor on 0x68 address
 */
void findMPU(int mpu_addr) {
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);

  if (data == 0) {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  } else {
    Serial.println("Dispositivo não encontrado!");
  }
}

/*
 * This function verifies if the sensor is active
 */
void checkMPU(int mpu_addr) {
  findMPU(MPU_ADDR);

  int data = readRegMPU(WHO_AM_I);  // Register 117 – Who Am I - 0x75

  if (data == 104) {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    data = readRegMPU(PWR_MGMT_1);  // Register 107 – Power Management 1-0x6B

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
 * This function initialize the sensor
 */
void initMPU() {
  setSleepOff();
  setGyroScale();
  setAccelScale();
}

/*
 * This function configures the sleep bit
 * Writes 0 on battery management register, putting the sensor on ACTIVE mode 
 */
void setSleepOff() {
  writeRegMPU(PWR_MGMT_1, 0);
}

/*
 * This function configures the Gyroscope scale
 * Gyroscope scale register: 0x1B [4:3]
 * 0 -> 250°/s
 *
 * FS_SEL  Full Scale Range
 *   0        ± 250 °/s      0b00000000
 *   1        ± 500 °/s      0b00001000
 *   2        ± 1000 °/s     0b00010000
 *   3        ± 2000 °/s     0b00011000
 */
void setGyroScale() {
  writeRegMPU(GYRO_CONFIG, 0);
}

/**
 * This function setup the accelerometer scales
 * Scale register: 0x1C [4:3]
 * 0 -> 250°/s
 *
 * AFS_SEL   Full Scale Range
 *    0           ± 2g            0b00000000
 *    1           ± 4g            0b00001000
 *    2           ± 8g            0b00010000
 *    3           ± 16g           0b00011000
 */
void setAccelScale() {
  writeRegMPU(ACCEL_CONFIG, 0);
}

/**
  * This function reads sensor raw data
  * In total, 14 bytes are read, being:
  * - 2 Bytes for each axis for Acelerometer (6 in total)
  * - 2 Bytes for each axis for Gyroscope    (6 in total)
  * - 2 Bytes for temperature
  */
void readRawMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

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

  int positionThreshold = 20;

  if (roll > positionThreshold) {
    Serial.println("Cima");
  } else if (roll < -positionThreshold) {
    Serial.println("Baixo");
  } else {
    Serial.println("Horizontal");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("nIniciando configuração do MPU6050n");
  initI2C();
  initMPU();
  checkMPU(MPU_ADDR);

  Serial.println("nConfiguração finalizada, iniciando loopn");
}

void loop() {
  readRawMPU();
  delay(200);
}
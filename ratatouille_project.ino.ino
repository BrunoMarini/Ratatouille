//Programa: Acelerometro com ESP8266 NodeMCU
#include <ESP8266WiFi.h>  // biblioteca para usar as funções de Wifi do módulo ESP8266
#include <Wire.h>         // biblioteca de comunicação I2C
#include <ArduinoJson.h>  // biblioteca JSON para sistemas embarcados

/*
 * Definições de alguns endereços mais comuns do MPU6050
 * os registros podem ser facilmente encontrados no mapa de registros do MPU6050
 */
const int MPU_ADDR = 0x68;      // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I = 0x75;      // registro de identificação do dispositivo
const int PWR_MGMT_1 = 0x6B;    // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG = 0x1B;   // registro de configuração do giroscópio
const int ACCEL_CONFIG = 0x1C;  // registro de configuração do acelerômetro
const int ACCEL_XOUT = 0x3B;    // registro de leitura do eixo X do acelerômetro

const int sda_pin = D5;  // definição do pino I2C SDA
const int scl_pin = D6;  // definição do pino I2C SCL

bool led_state = false;

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

bool bracoLevantado = false;

/*
 * função que configura a I2C com os pinos desejados 
 * sda_pin -> D5
 * scl_pin -> D6
 */
void initI2C() {
  Serial.println("initI2C()");
  Wire.begin(sda_pin, scl_pin);
}

/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val)  //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);  // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                   // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                   // escreve o valor no registro
  Wire.endTransmission(true);        // termina a transmissão
}

/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)  // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);  // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                   // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);       // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);     // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                // lê o byte e guarda em 'data'
  return data;                       //retorna 'data'
}

/*
 * função que procura pelo sensor no endereço 0x68
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
 * função que verifica se o sensor responde e se está ativo
 */
void checkMPU(int mpu_addr) {
  findMPU(MPU_ADDR);

  int data = readRegMPU(WHO_AM_I);  // Register 117 – Who Am I - 0x75

  if (data == 104) {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    data = readRegMPU(PWR_MGMT_1);  // Register 107 – Power Management 1-0x6B

    if (data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!");
  } else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}

/*
 * função de inicialização do sensor
 */
void initMPU() {
  setSleepOff();
  setGyroScale();
  setAccelScale();
}

/* 
 *  função para configurar o sleep bit  
 */
void setSleepOff() {
  writeRegMPU(PWR_MGMT_1, 0);  // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}

/* função para configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s

    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale() {
  writeRegMPU(GYRO_CONFIG, 0);
}

/**
 * This function setup the accelerometer scales
 * Scale register: 0x1C [4:3] -> 0 is 250°/s
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
  Wire.beginTransmission(MPU_ADDR);  // Starts the communication with MPU-6050
  Wire.write(ACCEL_XOUT);            // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);       // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);    // configura para receber 14 bytes começando do registro escolhido acima (0x3B)

  AcX = Wire.read() << 8;  // Read most significant byte first
  AcX |= Wire.read();      // OR with less significant byte
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();

  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();

  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read();

  // Converte aceleração para 'g'
  float ax_g = AcX / 16384.0;
  float ay_g = AcY / 16384.0;
  float az_g = AcZ / 16384.0;

  // Calcula ângulo aproximado em graus
  // float pitch = atan2(ay_g, az_g) * 180.0 / PI;
  float roll = atan2(ax_g, az_g) * 180.0 / PI;  // se quiser lateral

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
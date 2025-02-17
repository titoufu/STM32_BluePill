#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"

// Definição das variáveis globais
bool DMPReady = false;
uint8_t devStatus = 0;
uint16_t packetSize = 0;
unsigned long lastMillis = 0;

uint8_t FIFOBuffer[64];  // FIFO storage buffer
Quaternion q;            // [w, x, y, z]
VectorInt16 ac;          // Acelerômetro
VectorInt16 gc;          // Giroscópio
VectorInt16 gf;          // Leitura sem gravidade (se utilizada)
VectorInt16 aaWorld;     // Leitura no referencial mundo (se utilizada)
VectorFloat gravity;     // Vetor de gravidade
float euler[3] = { 0 };  // Ângulos de Euler
float ypr[3] = { 0 };    // Yaw, Pitch, Roll
float GyroScala = 0;
float AcelScala = 0;

unsigned long dt = 20;  // Tempo de amostragem (ms)



//
// Funções de inicialização
//

void inicializarMPU() {
  Serial.println("Inicializando MPU ...");

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  mpu.initialize();
  mpu.setFullScaleGyroRange(0);
  mpu.setFullScaleAccelRange(3);
  // Inicialize o DMP
  devStatus = mpu.dmpInitialize();
  Serial.print("acelRange=>>> ");
  Serial.println(mpu.getFullScaleAccelRange());
  Serial.print("gyroRange=>>>> ");
  Serial.println(mpu.getFullScaleGyroRange());
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("  ----   DMP CONNECTED   ----\n"));
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void calibrar() {
  Serial.println("Iniciando calibração...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(2);
  mpu.CalibrateGyro(2);
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  mpu.PrintActiveOffsets();
}

void gyroAccelGains(float &GyroScala, float &AcelScala) {
  // Configuração da escala do acelerômetro
  int8_t accelRange = mpu.getFullScaleAccelRange();
  switch (accelRange) {
    case MPU6050_ACCEL_FS_2:
      Serial.println("Faixa de aceleração: ±2g");
      AcelScala = 16384.0;
      break;
    case MPU6050_ACCEL_FS_4:
      Serial.println("Faixa de aceleração: ±4g");
      AcelScala = 8192.0;
      break;
    case MPU6050_ACCEL_FS_8:
      Serial.println("Faixa de aceleração: ±8g");
      AcelScala = 4096.0;
      break;
    case MPU6050_ACCEL_FS_16:
      Serial.println("Faixa de aceleração: ±16g");
      AcelScala = 2048.0;
      break;
    default:
      Serial.println("Faixa de aceleração desconhecida");
      AcelScala = 16384.0;
  }

  // Configuração da escala do giroscópio
  int8_t gyroRange = mpu.getFullScaleGyroRange();
  switch (gyroRange) {
    case MPU6050_GYRO_FS_250:
      Serial.println("Faixa do giroscópio: ±250 graus/segundo");
      GyroScala = 131.0;
      break;
    case MPU6050_GYRO_FS_500:
      Serial.println("Faixa do giroscópio: ±500 graus/segundo");
      GyroScala = 65.5;
      break;
    case MPU6050_GYRO_FS_1000:
      Serial.println("Faixa do giroscópio: ±1000 graus/segundo");
      GyroScala = 32.8;
      break;
    case MPU6050_GYRO_FS_2000:
      Serial.println("Faixa do giroscópio: ±2000 graus/segundo");
      GyroScala = 16.4;
      break;
    default:
      Serial.println("Faixa do giroscópio desconhecida");
  }
}

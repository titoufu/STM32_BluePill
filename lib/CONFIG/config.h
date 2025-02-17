#ifndef CONFIG_H
#define CONFIG_H

#define AIn1 PA12
#define AIn2 PA11
#define APWM PA2
#define StandbyPin PB12
#define OffsetSpeed 1

#define BIn1 PB6
#define BIn2 PB5
#define BPWM PA1


#define LED_VD PA5
#define LED_AZ PA4
#define LED_VM PA3

#define SCL_PIN PB10
#define SDA_PIN PB11

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID.h"
//#include "KalmanFilter.h"

// Declaração de variáveis globais (extern)
// Estas variáveis serão definidas em config.cpp

extern bool DMPReady;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern unsigned long lastMillis;

extern uint8_t FIFOBuffer[64];  // Buffer FIFO
extern Quaternion q;            // Quaternion [w, x, y, z]
extern VectorInt16 ac;          // Leituras do acelerômetro
extern VectorInt16 gc;          // Leituras do giroscópio
extern VectorInt16 gf;          // Leituras sem gravidade (se utilizadas)
extern VectorInt16 aaWorld;     // Leituras no referencial mundo (se utilizadas)
extern VectorFloat gravity;     // Vetor da gravidade
extern float euler[3];          // Ângulos de Euler [psi, theta, phi]
extern float ypr[3];            // Yaw, Pitch e Roll
extern float GyroScala;         // Escala do giroscópio
extern float AcelScala;         // Escala do acelerômetro

extern unsigned long dt;        // Tempo de amostragem em ms

// Objetos globais
extern PID myPID;
extern MPU6050 mpu;

// Declaração das funções de inicialização
void inicializarMPU();
void calibrar();
void gyroAccelGains(float &GyroScala, float &AcelScala);

#endif

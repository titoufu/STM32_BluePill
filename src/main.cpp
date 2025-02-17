#include "config.h"
#include "SparkFun_TB6612.h"

int uc=0;
const float umax = 1;           // Valor máximo esperado da saída do PID
const float maxSpeedSteps = 4000; // Velocidade máxima do motor em passos por segundo

// Variáveis do IMU
float A_x, A_y, A_z, G_x, G_y, G_z;
float roll_accel, roll_gyro, roll_filtered;
float alpha = 0.98;
unsigned long last_time = 0;

// Criação dos objetos globais e locais
// Criação do objeto AccelStepper para interface do tipo DRIVER
float Ku = 15.;
PID myPID(Ku, 0.001 * Ku, 0.00000 * Ku, -umax, umax); // (kp, ki, kd, -umax, +umax)
////

MPU6050 mpu;
// Criando os motores
Motor motorA = Motor(AIn1, AIn2, APWM, OffsetSpeed, StandbyPin);
Motor motorB = Motor(BIn1, BIn2, BPWM, OffsetSpeed, StandbyPin);
// KalmanFilter filtro;
//  the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_VD, OUTPUT);
  Serial.begin(115200);

  ////////////////////////////////////
  inicializarMPU();
  bool calibrarFlag = false; // flag de calibraça. faça verdadeiro se desja calibrar
  if (calibrarFlag)
  {
    calibrar();
  }
  else
  {
    //-2848.00000, -3235.00000, 882.00000, 62.00000, -164.00000, -4.00000
    // -1705.00000,   -312.00000,     2304.00000,     43.00000,       69.00000,       -50.00000
    //-747.00000,    -394.00000,     1918.00000,     83.00000,       17.00000,       24.00000
    
    mpu.setXAccelOffset(-747);
    mpu.setYAccelOffset(-394);
    mpu.setZAccelOffset(1918);

    mpu.setXGyroOffset(83);
    mpu.setYGyroOffset(17);
    mpu.setZGyroOffset(24);
  }

  //
  myPID.setSetpoint(0.006);

  // testando os motodes

  //
}

// the loop function runs over and over again forever
void loop()
{
  float A_x, A_y, A_z, G_x, G_y, G_z;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
  {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gc, FIFOBuffer);

    if ((millis() - lastMillis) >= dt)
    {
      lastMillis = millis();
      float ucTemp = myPID.compute(ypr[1], dt / 1000.);
      uc=ucTemp*255;
      Serial.print(uc);
      Serial.print("\t,\t");
      Serial.println(ypr[1] * 180 / PI);
    }
  }
 motorA.drive(uc) ;
 motorB.drive(-uc);

}
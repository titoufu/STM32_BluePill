

#ifndef PID_H
#define PID_H

class PID {
public:
  // Construtor
  PID(float kp, float ki, float kd, float outMin, float outMax);

  // Configura o setpoint desejado
  void setSetpoint(float setpoint);

  // Calcula a saída do controlador com base na entrada atual e no intervalo de tempo
  float compute(float input, float dt);

  // Configura os ganhos do controlador
  void setTunings(float kp, float ki, float kd);

  // Configura os limites da saída para evitar saturação
  void setOutputLimits(float min, float max);

  // Reinicia o controlador (zera o termo integral)
  void reset();

private:
  float kp;                   // Ganho proporcional
  float ki;                   // Ganho integral
  float kd;                   // Ganho derivativo
  float setpoint;             // Valor desejado
  float outMin;               // Limite mínimo da saída
  float outMax;               // Limite máximo da saída
  float integral;             // Termo integral acumulado
  float previousInput = 0.0;  // Valor da entrada anterior
  bool firstCompute = true;   // Flag para a primeira computação
  float previousDerivative = 0.0;  // Novo atributo para suavização
  float alpha = 0.2;               // Fator de suavização (ajustável)
};

#endif  // PID_H

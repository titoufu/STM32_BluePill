#include "PID.h"

// Construtor
PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax), integral(0), previousInput(0), firstCompute(true) {}

// Configura o setpoint desejado
void PID::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

// Calcula a saída do controlador com base na entrada atual e no intervalo de tempo
float PID::compute(float input, float dt) {
    // Calcula o erro
    float error = setpoint - input;

    // Termo proporcional
    float proportional = kp * error;

    // Termo integral
    integral += ki * error * dt;

    // Limita o termo integral para evitar "windup"
    if (integral > outMax) integral = outMax;
    else if (integral < outMin) integral = outMin;

    // Termo derivativo suavizado
    float derivative = 0;
    if (!firstCompute) {
        float derivadaAtual = (input - previousInput) / dt;
        // Suavização da derivada usando filtro exponencial
        derivative = alpha * derivadaAtual + (1 - alpha) * previousDerivative;
        previousDerivative = derivative;  // Armazena derivada suavizada
        derivative *= kd;  // Aplica ganho derivativo
    } else {
        firstCompute = false;
    }

    // Calcula a saída (com derivada suavizada)
    float output = proportional + integral - derivative;

    // Limita a saída para evitar saturação
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    // Armazena o valor da entrada atual para uso futuro
    previousInput = input;

    return output;
}


// Configura os ganhos do controlador
void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Configura os limites da saída para evitar saturação
void PID::setOutputLimits(float min, float max) {
    if (min >= max) return;
    outMin = min;
    outMax = max;

    // Também limita o termo integral
    if (integral > outMax) integral = outMax;
    else if (integral < outMin) integral = outMin;
}

// Reinicia o controlador (zera o termo integral)
void PID::reset() {
    integral = 0;
    previousInput = 0;
    firstCompute = true;
}

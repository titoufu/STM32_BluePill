#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
private:
    float Q_angle; // Variância do ruído do processo para o ângulo
    float Q_bias;  // Variância do ruído do processo para o viés do giroscópio
    float R_measure; // Variância do ruído da medida

    float angle;    // Ângulo estimado
    float bias;     // Viés estimado do giroscópio
    float rate;     // Velocidade angular sem viés

    float P[2][2];  // Matriz de covariância do erro

public:
    KalmanFilter(float Q_angle = 0.001, float Q_bias = 0.01, float R_measure = 0.001) {
        this->Q_angle = Q_angle;
        this->Q_bias = Q_bias;
        this->R_measure = R_measure;

        angle = 0.0;
        bias = 0.0;
        rate = 0.0;

        // Inicializa a matriz de covariância como zero
        P[0][0] = 0.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 0.0;
    }

    float update(float new_angle, float new_rate, float dt) {
        // Previsão do estado
        rate = new_rate - bias;
        angle += dt * rate;

        // Atualização da matriz de covariância
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Cálculo do ganho de Kalman
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Atualização com a medida
        float y = new_angle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // Atualização da matriz de covariância
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

#endif

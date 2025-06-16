#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct {
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman_t;

static inline void Kalman_Init(Kalman_t *k){
  k->Q_angle   = 0.005f;
  k->Q_bias    = 0.002f;
  k->R_measure = 0.03f;
  k->bias      = 0.0f;
  k->P[0][0] = k->P[1][1] = k->P[0][1] = k->P[1][0] = 0.0f;
}

static inline float Kalman_GetAngle(Kalman_t *k, float newAngle,
                                    float newRate, float dt){
  /* Predict */
  k->rate  = newRate - k->bias;
  k->angle += dt * k->rate;
  k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += k->Q_bias * dt;

  /* Update */
  float S  = k->P[0][0] + k->R_measure;
  float K0 = k->P[0][0] / S;
  float K1 = k->P[1][0] / S;
  float y  = newAngle - k->angle;
  k->angle += K0 * y;
  k->bias  += K1 * y;
  float P00_temp = k->P[0][0];
  float P01_temp = k->P[0][1];
  k->P[0][0] -= K0 * P00_temp;
  k->P[0][1] -= K0 * P01_temp;
  k->P[1][0] -= K1 * P00_temp;
  k->P[1][1] -= K1 * P01_temp;

  return k->angle;
}
#endif

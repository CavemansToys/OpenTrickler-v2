/**
 * @file gp_lite.h
 * @brief Lightweight Gaussian Process (ported from Pico)
 *
 * Optimized for:
 * - Small datasets (max 20 points)
 * - 2D parameter space (Kp, Kd)
 * - Squared exponential kernel
 * - UCB acquisition function
 */

#ifndef GP_LITE_H_
#define GP_LITE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GP_MAX_POINTS 20
#define GP_PARAM_DIM 2

typedef struct {
    float params[GP_PARAM_DIM];
    float score;
} gp_observation_t;

typedef struct {
    gp_observation_t obs[GP_MAX_POINTS];
    uint8_t n_obs;

    float length_scale;
    float signal_var;
    float noise_var;

    float K_inv[GP_MAX_POINTS][GP_MAX_POINTS];
    float alpha[GP_MAX_POINTS];
    bool matrices_valid;

    float param_min[GP_PARAM_DIM];
    float param_max[GP_PARAM_DIM];

    float beta;
} gp_model_t;

void gp_init(gp_model_t* gp, float kp_min, float kp_max, float kd_min, float kd_max);
bool gp_add_observation(gp_model_t* gp, float kp, float kd, float score);
void gp_predict(gp_model_t* gp, float kp, float kd, float* mean, float* variance);
float gp_ucb(gp_model_t* gp, float kp, float kd);
void gp_get_next_params(gp_model_t* gp, float* kp, float* kd);
void gp_get_best_observed(gp_model_t* gp, float* kp, float* kd, float* score);
void gp_reset(gp_model_t* gp);

#ifdef __cplusplus
}
#endif

#endif // GP_LITE_H_

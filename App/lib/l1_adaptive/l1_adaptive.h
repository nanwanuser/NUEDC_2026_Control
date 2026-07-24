#ifndef L1_ADAPTIVE_H
#define L1_ADAPTIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float sample_time_s;
    float speed_scale_rpm;
    float output_limit;
    float reference_bandwidth_rad_s;
    float control_filter_bandwidth_rad_s;
    float adaptation_gain;
    float uncertainty_limit;
} l1_adaptive_config_t;

typedef struct {
    l1_adaptive_config_t config;
    float reference_model_norm;
    float predictor_norm;
    float uncertainty_estimate;
    float adaptive_control_norm;
    float output;
    float model_decay;
    float filter_decay;
    uint8_t initialized;
    uint8_t output_saturated;
} l1_adaptive_controller_t;

void L1Adaptive_GetDefaultConfig(l1_adaptive_config_t *config);
uint8_t L1Adaptive_Init(l1_adaptive_controller_t *controller,
                        const l1_adaptive_config_t *config);
void L1Adaptive_Reset(l1_adaptive_controller_t *controller,
                      float measured_speed_rpm);
float L1Adaptive_Update(l1_adaptive_controller_t *controller,
                        float reference_speed_rpm,
                        float measured_speed_rpm);

#ifdef __cplusplus
}
#endif

#endif /* L1_ADAPTIVE_H */

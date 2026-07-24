#include "l1_adaptive.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define L1_ADAPTIVE_TWO_PI (6.2831853071795864769f)

static float L1Adaptive_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t L1Adaptive_ConfigIsValid(const l1_adaptive_config_t *config)
{
    if (config == NULL) {
        return 0U;
    }

    if (!isfinite(config->sample_time_s) ||
        !isfinite(config->speed_scale_rpm) ||
        !isfinite(config->output_limit) ||
        !isfinite(config->reference_bandwidth_rad_s) ||
        !isfinite(config->control_filter_bandwidth_rad_s) ||
        !isfinite(config->adaptation_gain) ||
        !isfinite(config->uncertainty_limit)) {
        return 0U;
    }

    return (config->sample_time_s > 0.0f) &&
           (config->speed_scale_rpm > 0.0f) &&
           (config->output_limit > 0.0f) &&
           (config->reference_bandwidth_rad_s > 0.0f) &&
           (config->control_filter_bandwidth_rad_s > 0.0f) &&
           (config->adaptation_gain > 0.0f) &&
           (config->uncertainty_limit > 0.0f);
}

void L1Adaptive_GetDefaultConfig(l1_adaptive_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->sample_time_s = 0.01f;
    config->speed_scale_rpm = 500.0f;
    config->output_limit = 1000.0f;
    config->reference_bandwidth_rad_s = 5.0f * L1_ADAPTIVE_TWO_PI;
    config->control_filter_bandwidth_rad_s = 10.0f * L1_ADAPTIVE_TWO_PI;
    config->adaptation_gain = 20.0f;
    config->uncertainty_limit = 1.5f;
}

uint8_t L1Adaptive_Init(l1_adaptive_controller_t *controller,
                        const l1_adaptive_config_t *config)
{
    float model_input_gain;

    if ((controller == NULL) || !L1Adaptive_ConfigIsValid(config)) {
        return 0U;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->model_decay = expf(-config->reference_bandwidth_rad_s *
                                   config->sample_time_s);
    controller->filter_decay = expf(-config->control_filter_bandwidth_rad_s *
                                    config->sample_time_s);

    model_input_gain = 1.0f - controller->model_decay;
    if (model_input_gain <= 0.0f) {
        memset(controller, 0, sizeof(*controller));
        return 0U;
    }

    controller->initialized = 1U;
    return 1U;
}

void L1Adaptive_Reset(l1_adaptive_controller_t *controller,
                      float measured_speed_rpm)
{
    float measured_norm;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        !isfinite(measured_speed_rpm)) {
        return;
    }

    measured_norm = measured_speed_rpm / controller->config.speed_scale_rpm;
    controller->reference_model_norm = measured_norm;
    controller->predictor_norm = measured_norm;
    controller->uncertainty_estimate = 0.0f;
    controller->adaptive_control_norm = 0.0f;
    controller->output = 0.0f;
    controller->output_saturated = 0U;
}

float L1Adaptive_Update(l1_adaptive_controller_t *controller,
                        float reference_speed_rpm,
                        float measured_speed_rpm)
{
    float raw_reference_norm;
    float reference_norm;
    float measured_norm;
    float prediction_error;
    float raw_output_norm;
    float output_norm;
    float model_input_gain;
    float filter_input_gain;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        !isfinite(reference_speed_rpm) || !isfinite(measured_speed_rpm)) {
        return 0.0f;
    }

    raw_reference_norm = reference_speed_rpm / controller->config.speed_scale_rpm;
    reference_norm = L1Adaptive_Clamp(raw_reference_norm, -1.0f, 1.0f);
    measured_norm = measured_speed_rpm / controller->config.speed_scale_rpm;
    prediction_error = controller->predictor_norm - measured_norm;

    controller->uncertainty_estimate -=
        controller->config.adaptation_gain * prediction_error *
        controller->config.sample_time_s;
    controller->uncertainty_estimate =
        L1Adaptive_Clamp(controller->uncertainty_estimate,
                         -controller->config.uncertainty_limit,
                         controller->config.uncertainty_limit);

    filter_input_gain = 1.0f - controller->filter_decay;
    controller->adaptive_control_norm =
        controller->filter_decay * controller->adaptive_control_norm -
        filter_input_gain * controller->uncertainty_estimate;
    controller->adaptive_control_norm =
        L1Adaptive_Clamp(controller->adaptive_control_norm, -1.0f, 1.0f);

    raw_output_norm = reference_norm + controller->adaptive_control_norm;
    output_norm = L1Adaptive_Clamp(raw_output_norm, -1.0f, 1.0f);
    controller->output_saturated =
        ((raw_reference_norm != reference_norm) || (raw_output_norm != output_norm)) ? 1U : 0U;
    controller->output = output_norm * controller->config.output_limit;

    model_input_gain = 1.0f - controller->model_decay;
    controller->reference_model_norm =
        controller->model_decay * controller->reference_model_norm +
        model_input_gain * reference_norm;
    controller->predictor_norm =
        controller->model_decay * controller->predictor_norm +
        model_input_gain * (output_norm + controller->uncertainty_estimate);

    return controller->output;
}

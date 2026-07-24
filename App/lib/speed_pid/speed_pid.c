#include "speed_pid.h"

#include <stddef.h>

#define SPEED_PID_TWO_PI_F 6.28318530717958647692f

static float speed_pid_clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float speed_pid_update_derivative(speed_pid_controller *controller,
                                         float measurement,
                                         float sample_period_s) {
    float derivative = -(measurement - controller->previous_measurement) /
                       sample_period_s;
    float cutoff_hz = controller->params.derivative_cutoff_hz;

    controller->previous_measurement = measurement;
    if (cutoff_hz <= 0.0f) {
        controller->filtered_derivative = derivative;
        return derivative;
    }

    float factor = SPEED_PID_TWO_PI_F * cutoff_hz * sample_period_s;
    float alpha = factor / (1.0f + factor);
    controller->filtered_derivative +=
        alpha * (derivative - controller->filtered_derivative);
    return controller->filtered_derivative;
}

void speed_pid_init(speed_pid_controller *controller,
                    const speed_pid_params *params) {
    if (controller == NULL || params == NULL) {
        return;
    }

    controller->params = *params;
    speed_pid_reset(controller);
}

void speed_pid_reset(speed_pid_controller *controller) {
    if (controller == NULL) {
        return;
    }

    controller->integral = 0.0f;
    controller->previous_measurement = 0.0f;
    controller->filtered_derivative = 0.0f;
    controller->initialized = 0U;
}

float speed_pid_update(speed_pid_controller *controller,
                       float target_rpm,
                       float measured_rpm,
                       float sample_period_s) {
    float error;
    float candidate_integral;
    float derivative;
    float candidate_output;

    if (controller == NULL || sample_period_s <= 0.0f) {
        return 0.0f;
    }
    if (controller->initialized == 0U) {
        controller->previous_measurement = measured_rpm;
        controller->initialized = 1U;
    }

    error = target_rpm - measured_rpm;
    derivative = speed_pid_update_derivative(controller, measured_rpm,
                                             sample_period_s);
    candidate_integral = speed_pid_clamp(
        controller->integral + controller->params.ki * error * sample_period_s,
        controller->params.integral_min, controller->params.integral_max);
    candidate_output = controller->params.kp * error + candidate_integral +
                       controller->params.kd * derivative;

    if ((candidate_output <= controller->params.output_max || error < 0.0f) &&
        (candidate_output >= controller->params.output_min || error > 0.0f)) {
        controller->integral = candidate_integral;
    }

    return speed_pid_clamp(controller->params.kp * error +
                           controller->integral +
                           controller->params.kd * derivative,
                           controller->params.output_min,
                           controller->params.output_max);
}

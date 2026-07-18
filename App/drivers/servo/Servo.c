/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Servo driver source (first-order smooth motion)
  ******************************************************************************
  */

#include "Servo.h"
#include <math.h>

Servo_t g_servo;

static float Clamp_Float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float Clamp_Angle(float angle)
{
    return Clamp_Float(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

static uint32_t Angle_To_Pulse(float angle)
{
    float pulse;

    angle = Clamp_Angle(angle);
    pulse = SERVO_MIN_PULSE +
            (angle - SERVO_MIN_ANGLE) *
            (SERVO_MAX_PULSE - SERVO_MIN_PULSE) /
            (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    return (uint32_t)(pulse + 0.5f);
}

static void Servo_WriteAngle(float angle)
{
    __HAL_TIM_SET_COMPARE(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL, Angle_To_Pulse(angle));
}

void Servo_Init(void)
{
    g_servo.current_angle = 0.0f;
    g_servo.target_angle = 0.0f;
    g_servo.current_velocity = 0.0f;
    g_servo.max_velocity = 0.0f;
    g_servo.acceleration = 0.0f;
    g_servo.filter_time = SERVO_MIN_FILTER_TIME;
    g_servo.last_tick = HAL_GetTick();
    g_servo.is_moving = 0;

    Servo_WriteAngle(g_servo.current_angle);
    HAL_TIM_PWM_Start(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
}

void Servo_SetAngle_Direct(float angle)
{
    angle = Clamp_Angle(angle);

    g_servo.target_angle = angle;
    g_servo.current_angle = angle;
    g_servo.current_velocity = 0.0f;
    g_servo.is_moving = 0;
    g_servo.last_tick = HAL_GetTick();

    Servo_WriteAngle(angle);
}

/**
  * @brief  Start first-order smooth motion.
  * @param  target_angle Target angle in degrees.
  * @param  max_speed    Maximum speed in degrees per second.
  * @param  accel        Reference acceleration in degrees per second squared.
  *
  * The velocity command is limited by max_speed and filtered with a first-order
  * filter. accel is converted to a time constant: tau = max_speed / accel.
  */
void Servo_SetAngle_Smooth(float target_angle, float max_speed, float accel)
{
    float distance;

    target_angle = Clamp_Angle(target_angle);
    max_speed = fabsf(max_speed);
    accel = fabsf(accel);

    if (max_speed <= 0.0f) {
        Servo_SetAngle_Direct(target_angle);
        return;
    }

    g_servo.target_angle = target_angle;
    g_servo.max_velocity = max_speed;
    g_servo.acceleration = accel;

    if (accel > 0.0f) {
        g_servo.filter_time = max_speed / accel;
    } else {
        g_servo.filter_time = 0.20f;
    }
    g_servo.filter_time = Clamp_Float(g_servo.filter_time,
                                      SERVO_MIN_FILTER_TIME,
                                      SERVO_MAX_FILTER_TIME);

    distance = fabsf(g_servo.target_angle - g_servo.current_angle);
    if (distance > SERVO_ARRIVAL_ANGLE) {
        if (!g_servo.is_moving) {
            g_servo.last_tick = HAL_GetTick();
        }
        g_servo.is_moving = 1;
    } else {
        Servo_SetAngle_Direct(target_angle);
    }
}

/**
  * @brief  First-order smooth motion loop.
  * @note   Call this frequently in while(1) or a timer interrupt.
  */
void Servo_Loop_Process(void)
{
    uint32_t current_tick;
    float dt;
    float error;
    float distance;
    float target_velocity;
    float alpha;
    float step;

    if (!g_servo.is_moving) return;

    current_tick = HAL_GetTick();
    dt = (float)(current_tick - g_servo.last_tick) / 1000.0f;

    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.02f;

    g_servo.last_tick = current_tick;

    error = g_servo.target_angle - g_servo.current_angle;
    distance = fabsf(error);

    if (distance <= SERVO_ARRIVAL_ANGLE &&
        fabsf(g_servo.current_velocity) <= (g_servo.max_velocity * 0.02f + 0.1f)) {
        g_servo.current_angle = g_servo.target_angle;
        g_servo.current_velocity = 0.0f;
        g_servo.is_moving = 0;
        Servo_WriteAngle(g_servo.current_angle);
        return;
    }

    target_velocity = error / g_servo.filter_time;
    target_velocity = Clamp_Float(target_velocity,
                                  -g_servo.max_velocity,
                                  g_servo.max_velocity);

    alpha = dt / (g_servo.filter_time + dt);
    g_servo.current_velocity += (target_velocity - g_servo.current_velocity) * alpha;

    step = g_servo.current_velocity * dt;
    if (fabsf(step) >= distance) {
        g_servo.current_angle = g_servo.target_angle;
        g_servo.current_velocity = 0.0f;
        g_servo.is_moving = 0;
    } else {
        g_servo.current_angle += step;
    }

    Servo_WriteAngle(g_servo.current_angle);
}

void Servo_Stop(void)
{
    g_servo.target_angle = g_servo.current_angle;
    g_servo.current_velocity = 0.0f;
    g_servo.is_moving = 0;
}

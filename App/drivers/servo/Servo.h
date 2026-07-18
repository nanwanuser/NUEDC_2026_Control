/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Servo driver header (first-order smooth motion)
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Hardware mapping ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
#define SERVO_TIM_HANDLE      &htim1
#define SERVO_TIM_CHANNEL     TIM_CHANNEL_1

/* Servo physical parameters -------------------------------------------------*/
#define SERVO_MIN_ANGLE       0.0f
#define SERVO_MAX_ANGLE       180.0f
#define SERVO_MIN_PULSE       500
#define SERVO_MAX_PULSE       2500

/* First-order motion parameters --------------------------------------------*/
#define SERVO_ARRIVAL_ANGLE   0.05f
#define SERVO_MIN_FILTER_TIME 0.02f
#define SERVO_MAX_FILTER_TIME 1.50f

/* Servo smooth motion state -------------------------------------------------*/
typedef struct {
    float current_angle;    /* Current output angle (degree) */
    float target_angle;     /* Target angle (degree) */

    float current_velocity; /* Current filtered velocity (degree/s) */
    float max_velocity;     /* Velocity limit (degree/s) */
    float acceleration;     /* Reference acceleration (degree/s^2) */
    float filter_time;      /* First-order time constant (s) */

    uint32_t last_tick;     /* Last update time (ms) */
    uint8_t is_moving;      /* 1: moving, 0: reached */
} Servo_t;

extern Servo_t g_servo;

/* Public API ----------------------------------------------------------------*/
void Servo_Init(void);
void Servo_SetAngle_Direct(float angle);
void Servo_SetAngle_Smooth(float target_angle, float max_speed, float accel);
void Servo_Loop_Process(void);
void Servo_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */

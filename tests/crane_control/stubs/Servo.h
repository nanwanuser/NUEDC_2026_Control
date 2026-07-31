#ifndef TEST_SERVO_H
#define TEST_SERVO_H

#include "main.h"

#include <stdint.h>

#define SERVO_MIN_ANGLE_DEG 0.0f
#define SERVO_MAX_ANGLE_DEG 180.0f
#define SERVO_CENTER_ANGLE_DEG 90.0f

typedef enum {
    SERVO_LIFT = 0,
    SERVO_END_YAW
} Servo_Id_t;

HAL_StatusTypeDef Servo_Init(void);
HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg);
HAL_StatusTypeDef Servo_SetAngleImmediate(Servo_Id_t servo_id, float angle_deg);
void Servo_Update(void);
uint8_t Servo_IsAtTarget(Servo_Id_t servo_id);

#endif

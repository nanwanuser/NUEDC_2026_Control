/**
 * @file Servo.c
 * @brief MG996R 角度到 PWM 的直接线性映射实现。
 */
#include "Servo.h"

#define SERVO_TOTAL_ANGLE_STEPS  1800UL
#define SERVO_ARRIVAL_TOLERANCE  0.05f

static const Servo_Config_t g_servo_config[SERVO_COUNT] = {
    {&htim1, SERVO_LIFT_PWM_CHANNEL,
     (SERVO_MIN_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL,
     (SERVO_MAX_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL},
    {&htim1, SERVO_END_YAW_PWM_CHANNEL,
     (SERVO_MIN_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL,
     (SERVO_MAX_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL},
};

static Servo_State_t g_servo_state[SERVO_COUNT];

static float Servo_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float Servo_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float Servo_QuantizeAngle(float angle_deg)
{
    uint32_t angle_tenths;

    angle_deg = Servo_Clamp(angle_deg,
                            SERVO_MIN_ANGLE_DEG,
                            SERVO_MAX_ANGLE_DEG);
    angle_tenths = (uint32_t)(angle_deg * 10.0f + 0.5f);
    return (float)angle_tenths * SERVO_ANGLE_RESOLUTION_DEG;
}

static uint8_t Servo_IsValid(Servo_Id_t servo_id)
{
    return (uint32_t)servo_id < SERVO_COUNT ? 1U : 0U;
}

static uint32_t Servo_ChannelEnableMask(uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return TIM_CCER_CC1E;
    case TIM_CHANNEL_2:
        return TIM_CCER_CC2E;
    case TIM_CHANNEL_3:
        return TIM_CCER_CC3E;
    case TIM_CHANNEL_4:
        return TIM_CCER_CC4E;
    default:
        return 0U;
    }
}

/**
 * Z 轴按实际安装方向映射：0~180 deg 对应 2500~500 us。
 * 末端 Yaw 保持标准方向：0~180 deg 对应 500~2500 us。
 */
static uint32_t Servo_AngleToCompare(Servo_Id_t servo_id, float angle_deg)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];
    uint32_t angle_tenths;
    uint32_t compare_range;

    angle_deg = Servo_QuantizeAngle(angle_deg);
    angle_tenths = (uint32_t)(angle_deg * 10.0f + 0.5f);
    compare_range = config->max_compare - config->min_compare;

    if (servo_id == SERVO_LIFT) {
        return config->max_compare -
               (angle_tenths * compare_range +
                SERVO_TOTAL_ANGLE_STEPS / 2UL) /
                   SERVO_TOTAL_ANGLE_STEPS;
    }

    return config->min_compare +
           (angle_tenths * compare_range + SERVO_TOTAL_ANGLE_STEPS / 2UL) /
               SERVO_TOTAL_ANGLE_STEPS;
}

static void Servo_WriteAngle(Servo_Id_t servo_id, float angle_deg)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];

    __HAL_TIM_SET_COMPARE(config->timer,
                          config->channel,
                          Servo_AngleToCompare(servo_id, angle_deg));
}

static HAL_StatusTypeDef Servo_StartPwm(Servo_Id_t servo_id)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];
    uint32_t enable_mask = Servo_ChannelEnableMask(config->channel);

    if (config->timer == (TIM_HandleTypeDef *)0 ||
        config->timer->Instance == (TIM_TypeDef *)0 || enable_mask == 0U) {
        return HAL_ERROR;
    }

    if ((config->timer->Instance->CCER & enable_mask) != 0U) {
        return HAL_OK;
    }
    return HAL_TIM_PWM_Start(config->timer, config->channel);
}

static HAL_StatusTypeDef Servo_ApplyAngle(Servo_Id_t servo_id, float angle_deg)
{
    Servo_State_t *state;

    if (Servo_IsValid(servo_id) == 0U || angle_deg != angle_deg) {
        return HAL_ERROR;
    }

    state = &g_servo_state[servo_id];
    if (state->initialized == 0U) {
        return HAL_ERROR;
    }

    angle_deg = Servo_QuantizeAngle(angle_deg);
    state->target_angle_deg = angle_deg;
    state->current_angle_deg = angle_deg;
    Servo_WriteAngle(servo_id, angle_deg);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_Init(void)
{
    static const float init_angles[SERVO_COUNT] = {
        SERVO_LIFT_INIT_ANGLE_DEG,
        SERVO_END_YAW_INIT_ANGLE_DEG,
    };
    uint32_t index;

    for (index = 0U; index < SERVO_COUNT; ++index) {
        Servo_Id_t servo_id = (Servo_Id_t)index;
        Servo_State_t *state = &g_servo_state[index];
        float angle_deg = Servo_QuantizeAngle(init_angles[index]);

        state->target_angle_deg = angle_deg;
        state->current_angle_deg = angle_deg;
        state->initialized = 1U;
        Servo_WriteAngle(servo_id, angle_deg);

        if (Servo_StartPwm(servo_id) != HAL_OK) {
            state->initialized = 0U;
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg)
{
    return Servo_ApplyAngle(servo_id, angle_deg);
}

HAL_StatusTypeDef Servo_SetAngleImmediate(Servo_Id_t servo_id, float angle_deg)
{
    return Servo_ApplyAngle(servo_id, angle_deg);
}

HAL_StatusTypeDef Servo_SetAngles(float lift_angle_deg,
                                  float end_yaw_angle_deg)
{
    if (Servo_ApplyAngle(SERVO_LIFT, lift_angle_deg) != HAL_OK) {
        return HAL_ERROR;
    }
    return Servo_ApplyAngle(SERVO_END_YAW, end_yaw_angle_deg);
}

void Servo_Update(void)
{
}

void Servo_Stop(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U ||
        g_servo_state[servo_id].initialized == 0U) {
        return;
    }
    Servo_WriteAngle(servo_id, g_servo_state[servo_id].current_angle_deg);
}

void Servo_StopAll(void)
{
    Servo_Stop(SERVO_LIFT);
    Servo_Stop(SERVO_END_YAW);
}

float Servo_GetCurrentAngle(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return SERVO_CENTER_ANGLE_DEG;
    }
    return g_servo_state[servo_id].current_angle_deg;
}

float Servo_GetTargetAngle(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return SERVO_CENTER_ANGLE_DEG;
    }
    return g_servo_state[servo_id].target_angle_deg;
}

uint8_t Servo_IsAtTarget(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U ||
        g_servo_state[servo_id].initialized == 0U) {
        return 0U;
    }
    return Servo_Abs(g_servo_state[servo_id].target_angle_deg -
                     g_servo_state[servo_id].current_angle_deg) <=
                   SERVO_ARRIVAL_TOLERANCE
               ? 1U
               : 0U;
}

const Servo_State_t *Servo_GetState(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return (const Servo_State_t *)0;
    }
    return &g_servo_state[servo_id];
}

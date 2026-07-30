/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   GPIO active buzzer driver
  ******************************************************************************
  */

#include "buzzer.h"

static bool buzzer_config_is_valid(const buzzer_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if ((config->active_level != GPIO_PIN_RESET) &&
        (config->active_level != GPIO_PIN_SET)) {
        return false;
    }

    if (config->write != NULL) {
        return true;
    }

    if ((config->gpio_port == NULL) || (config->gpio_pin == 0U)) {
        return false;
    }

    return (config->gpio_pin & (uint16_t)(config->gpio_pin - 1U)) == 0U;
}

static void buzzer_write(buzzer_handle_t *buzzer, bool enabled)
{
    GPIO_PinState output_level;

    if (buzzer->config.write != NULL) {
        buzzer->config.write(enabled, buzzer->config.context);
    } else {
        output_level = enabled
                     ? buzzer->config.active_level
                     : (buzzer->config.active_level == GPIO_PIN_SET
                        ? GPIO_PIN_RESET
                        : GPIO_PIN_SET);
        HAL_GPIO_WritePin(buzzer->config.gpio_port,
                          buzzer->config.gpio_pin,
                          output_level);
    }

    buzzer->enabled = enabled;
}

bool buzzer_init(buzzer_handle_t *buzzer, const buzzer_config_t *config)
{
    if ((buzzer == NULL) || !buzzer_config_is_valid(config)) {
        return false;
    }

    buzzer->config = *config;
    buzzer->stop_tick = 0U;
    buzzer->enabled = false;
    buzzer->timed_beep = false;
    buzzer->initialized = true;

    /* 初始化时立即进入安全的关闭状态。 */
    buzzer_write(buzzer, false);
    return true;
}

void buzzer_set(buzzer_handle_t *buzzer, bool enabled)
{
    if ((buzzer == NULL) || !buzzer->initialized) {
        return;
    }

    buzzer->timed_beep = false;
    buzzer_write(buzzer, enabled);
}

void buzzer_on(buzzer_handle_t *buzzer)
{
    buzzer_set(buzzer, true);
}

void buzzer_off(buzzer_handle_t *buzzer)
{
    buzzer_set(buzzer, false);
}

void buzzer_toggle(buzzer_handle_t *buzzer)
{
    if ((buzzer == NULL) || !buzzer->initialized) {
        return;
    }

    buzzer_set(buzzer, !buzzer->enabled);
}

bool buzzer_beep(buzzer_handle_t *buzzer, uint32_t duration_ms)
{
    if ((buzzer == NULL) || !buzzer->initialized) {
        return false;
    }

    if (duration_ms == 0U) {
        buzzer_off(buzzer);
        return true;
    }

    buzzer->stop_tick = HAL_GetTick() + duration_ms;
    buzzer->timed_beep = true;
    buzzer_write(buzzer, true);
    return true;
}

void buzzer_process_at(buzzer_handle_t *buzzer, uint32_t now_ms)
{
    if ((buzzer == NULL) || !buzzer->initialized || !buzzer->timed_beep) {
        return;
    }

    /* 有符号差值比较可正确处理 HAL tick 回绕。 */
    if ((int32_t)(now_ms - buzzer->stop_tick) >= 0) {
        buzzer->timed_beep = false;
        buzzer_write(buzzer, false);
    }
}

void buzzer_process(buzzer_handle_t *buzzer)
{
    buzzer_process_at(buzzer, HAL_GetTick());
}

bool buzzer_is_on(const buzzer_handle_t *buzzer)
{
    if ((buzzer == NULL) || !buzzer->initialized) {
        return false;
    }

    return buzzer->enabled;
}

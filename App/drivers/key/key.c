/**
  ******************************************************************************
  * @file    key.c
  * @brief   Parameterized GPIO key driver
  ******************************************************************************
  */

#include "key.h"

static bool key_config_is_valid(const key_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if ((config->pressed_level != GPIO_PIN_RESET) &&
        (config->pressed_level != GPIO_PIN_SET)) {
        return false;
    }

    if (config->read_pressed != NULL) {
        return true;
    }

    if ((config->gpio_port == NULL) || (config->gpio_pin == 0U)) {
        return false;
    }

    /* HAL_GPIO_ReadPin 只应读取单个引脚。 */
    return (config->gpio_pin & (uint16_t)(config->gpio_pin - 1U)) == 0U;
}

static bool key_read_pressed(const key_handle_t *key)
{
    if (key->config.read_pressed != NULL) {
        return key->config.read_pressed(key->config.context);
    }

    return HAL_GPIO_ReadPin(key->config.gpio_port, key->config.gpio_pin) ==
           key->config.pressed_level;
}

bool key_init(key_handle_t *key, const key_config_t *config)
{
    uint32_t now_ms;
    bool pressed;

    if ((key == NULL) || !key_config_is_valid(config)) {
        return false;
    }

    key->config = *config;
    if (key->config.debounce_time_ms == 0U) {
        key->config.debounce_time_ms = KEY_DEFAULT_DEBOUNCE_TIME_MS;
    }

    now_ms = HAL_GetTick();
    pressed = key_read_pressed(key);

    key->raw_change_tick = now_ms;
    key->pressed_tick = now_ms;
    key->pending_events = KEY_EVENT_NONE;
    key->raw_pressed = pressed;
    key->stable_pressed = pressed;
    key->long_press_reported = false;
    key->initialized = true;

    return true;
}

void key_process_at(key_handle_t *key, uint32_t now_ms)
{
    bool pressed;

    if ((key == NULL) || !key->initialized) {
        return;
    }

    pressed = key_read_pressed(key);
    if (pressed != key->raw_pressed) {
        key->raw_pressed = pressed;
        key->raw_change_tick = now_ms;
    }

    /* 原始电平保持足够时间后，才更新稳定状态并产生边沿事件。 */
    if ((key->raw_pressed != key->stable_pressed) &&
        ((uint32_t)(now_ms - key->raw_change_tick) >= key->config.debounce_time_ms)) {
        key->stable_pressed = key->raw_pressed;

        if (key->stable_pressed) {
            key->pressed_tick = now_ms;
            key->long_press_reported = false;
            key->pending_events |= KEY_EVENT_PRESSED;
        } else {
            key->pending_events |= KEY_EVENT_RELEASED;
            key->long_press_reported = false;
        }
    }

    if (key->stable_pressed && !key->long_press_reported &&
        (key->config.long_press_time_ms > 0U) &&
        ((uint32_t)(now_ms - key->pressed_tick) >= key->config.long_press_time_ms)) {
        key->pending_events |= KEY_EVENT_LONG_PRESS;
        key->long_press_reported = true;
    }
}

void key_process(key_handle_t *key)
{
    key_process_at(key, HAL_GetTick());
}

void key_process_all(key_handle_t *keys, uint32_t key_count)
{
    uint32_t index;
    uint32_t now_ms;

    if (keys == NULL) {
        return;
    }

    now_ms = HAL_GetTick();
    for (index = 0U; index < key_count; index++) {
        key_process_at(&keys[index], now_ms);
    }
}

bool key_is_pressed(const key_handle_t *key)
{
    if ((key == NULL) || !key->initialized) {
        return false;
    }

    return key->stable_pressed;
}

key_event_t key_get_events(key_handle_t *key)
{
    key_event_t events;

    if ((key == NULL) || !key->initialized) {
        return KEY_EVENT_NONE;
    }

    events = (key_event_t)key->pending_events;
    key->pending_events = KEY_EVENT_NONE;
    return events;
}

void key_clear_events(key_handle_t *key)
{
    if (key != NULL) {
        key->pending_events = KEY_EVENT_NONE;
    }
}

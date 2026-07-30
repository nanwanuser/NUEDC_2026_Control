/**
  ******************************************************************************
  * @file    key.h
  * @brief   Parameterized GPIO key driver
  ******************************************************************************
  */

#ifndef APP_DRIVERS_KEY_KEY_H
#define APP_DRIVERS_KEY_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define KEY_DEFAULT_DEBOUNCE_TIME_MS  20U

typedef bool (*key_read_callback_t)(void *context);

typedef enum {
    KEY_EVENT_NONE       = 0U,
    KEY_EVENT_PRESSED    = (1U << 0),
    KEY_EVENT_RELEASED   = (1U << 1),
    KEY_EVENT_LONG_PRESS = (1U << 2)
} key_event_t;

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    GPIO_PinState pressed_level;
    uint32_t debounce_time_ms;
    uint32_t long_press_time_ms;

    /* 非空时优先使用回调，便于后续更换按键硬件或读取方式。 */
    key_read_callback_t read_pressed;
    void *context;
} key_config_t;

typedef struct {
    key_config_t config;
    uint32_t raw_change_tick;
    uint32_t pressed_tick;
    uint8_t pending_events;
    bool raw_pressed;
    bool stable_pressed;
    bool long_press_reported;
    bool initialized;
} key_handle_t;

bool key_init(key_handle_t *key, const key_config_t *config);
void key_process(key_handle_t *key);
void key_process_at(key_handle_t *key, uint32_t now_ms);
void key_process_all(key_handle_t *keys, uint32_t key_count);

bool key_is_pressed(const key_handle_t *key);
key_event_t key_get_events(key_handle_t *key);
void key_clear_events(key_handle_t *key);

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVERS_KEY_KEY_H */

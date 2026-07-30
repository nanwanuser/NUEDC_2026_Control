/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   GPIO active buzzer driver
  ******************************************************************************
  */

#ifndef APP_DRIVERS_BUZZER_BUZZER_H
#define APP_DRIVERS_BUZZER_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*buzzer_write_callback_t)(bool enabled, void *context);

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    GPIO_PinState active_level;

    /* 非空时优先使用回调，可在以后接入 PWM 或外部驱动电路。 */
    buzzer_write_callback_t write;
    void *context;
} buzzer_config_t;

typedef struct {
    buzzer_config_t config;
    uint32_t stop_tick;
    bool enabled;
    bool timed_beep;
    bool initialized;
} buzzer_handle_t;

bool buzzer_init(buzzer_handle_t *buzzer, const buzzer_config_t *config);
void buzzer_set(buzzer_handle_t *buzzer, bool enabled);
void buzzer_on(buzzer_handle_t *buzzer);
void buzzer_off(buzzer_handle_t *buzzer);
void buzzer_toggle(buzzer_handle_t *buzzer);

bool buzzer_beep(buzzer_handle_t *buzzer, uint32_t duration_ms);
void buzzer_process(buzzer_handle_t *buzzer);
void buzzer_process_at(buzzer_handle_t *buzzer, uint32_t now_ms);
bool buzzer_is_on(const buzzer_handle_t *buzzer);

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVERS_BUZZER_BUZZER_H */

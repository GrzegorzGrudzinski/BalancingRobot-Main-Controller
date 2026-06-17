#include "button.h"
#include "config.h"

#include <stdint.h>


static uint8_t button_pressed = 0;
static uint8_t button_prev_state = 0;
static uint32_t button_debounce_time = 0;

void ButtonUpdate() {
    uint8_t button_state = HAL_GPIO_ReadPin(BUTTON_PIN_PORT, BUTTON_PIN_NUM);

    if (button_state != button_prev_state) { 
        button_debounce_time = HAL_GetTick(); 
    }
    if (HAL_GetTick() - button_debounce_time > 50) { 
      static uint8_t button_debounced_state = GPIO_PIN_SET;

      if (button_state != button_debounced_state) {
        button_debounced_state = button_state; 
        
        if (button_debounced_state == BUTTON_PRESSED) {
            button_pressed = 1;
        }
      }     
    }
    button_prev_state = button_state;
}

uint8_t ButtonIsPressed(void) {
    return button_pressed;
}

void ButtonClear(void) {
    button_pressed = 0;
}
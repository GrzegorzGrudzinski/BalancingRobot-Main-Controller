#ifndef BUTTON_H
#define BUTTON_H

#include "config.h"
#include "main.h"
#include <stdint.h>

void ButtonUpdate(void);
uint8_t ButtonIsPressed(void);
void ButtonClear(void);

#endif
#ifndef PROCESS_COMMANDS_H
#define PROCESS_COMMANDS_H

#include "main.h"

void process_data(RX_Struct* rx_struct, const char *data, size_t length, UART_HandleTypeDef *huart);


#endif // PROCESS_COMMANDS_H
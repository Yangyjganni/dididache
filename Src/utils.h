#ifndef UTIL_H
#define UTIL_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

int fputc(int ch,FILE *f);

void setInitHandle(UART_HandleTypeDef* huart);

#endif

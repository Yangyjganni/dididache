#include "utils.h"


UART_HandleTypeDef *this_handle;

void setInitHandle(UART_HandleTypeDef* huart)
{
	this_handle = huart;
}

int fputc(int ch,FILE *f)
{
	uint8_t temp[1]={ch};
	HAL_UART_Transmit(this_handle,temp,1,100);
	return(ch);
}

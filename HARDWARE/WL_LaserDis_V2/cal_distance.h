#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "stdio.h"

void GPIO_Configuration(void);
int cal_distance1(void);
int cal_distance2(int last_flag);
int cal_distance3(void);
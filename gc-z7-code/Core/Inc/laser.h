#ifndef __LASER_H
#define __LASER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define LASER_GPIO_PORT GPIOE
#define LASER_GPIO_PIN  GPIO_PIN_14

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_Set(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif

#include "laser.h"

void Laser_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_GPIO_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LASER_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStruct);
}

void Laser_On(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_GPIO_PIN, GPIO_PIN_SET);
}

void Laser_Off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_GPIO_PIN, GPIO_PIN_RESET);
}

void Laser_Set(uint8_t enable)
{
    if (enable != 0U) {
        Laser_On();
    } else {
        Laser_Off();
    }
}

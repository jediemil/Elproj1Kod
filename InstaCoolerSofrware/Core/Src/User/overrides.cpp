//
// Created by emilr on 2025-04-27.
//

#include <main.h>
#include "main_cpp.h"
#include "stdio.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) // PE0 or PB0
    {
        // Check direction using second pin (you decide which is A and which is B)
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
            change_encoder(1); // Clockwise
        } else {
            change_encoder(-1); // Counter-clockwise
        }
    }
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        //__io_putchar(*ptr++);
        ITM_SendChar(*ptr++);
    }
    return len;
}
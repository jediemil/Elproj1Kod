//
// Created by emilr on 2025-04-27.
//

#include <main.h>
#include "main_cpp.h"
#include "stdio.h"
#include "tim.h"

uint64_t lastIRQTick = 0;

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    //if (getTimeTicks() < lastIRQTick + 5) return;
    lastIRQTick = getTimeTicks();
    if (GPIO_Pin == GPIO_PIN_13) // PE0 or PB0
    {
        // Check direction using second pin (you decide which is A and which is B)
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
            changeEncoderIt(1); // Clockwise
        } else {
            changeEncoderIt(-1); // Counter-clockwise
        }
    } else if(GPIO_Pin == GPIO_PIN_15) {
        buttonPressIt();
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

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

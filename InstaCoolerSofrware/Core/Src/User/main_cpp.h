#ifndef MAIN_CPP_H_
#define MAIN_CPP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <main.h>

int main_cpp();
void setRGB(uint8_t r, uint8_t g, uint8_t b);
uint64_t getTimeTicks();
void addTick();
void changeEncoderIt(int change);
void buttonPressIt();
void terminateMotorTask();
void startMotorTask();

#ifdef __cplusplus
}
#endif

#endif //MAIN_CPP_H_

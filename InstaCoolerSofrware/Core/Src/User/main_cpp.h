#ifndef MAIN_CPP_H_
#define MAIN_CPP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <main.h>

int main_cpp();
void setRGB(uint8_t r, uint8_t g, uint8_t b);
unsigned long getTimeTicks();
void addTick();
void changeEncoderIt(int change);
void buttonPressIt();

#ifdef __cplusplus
}
#endif

#endif //MAIN_CPP_H_

/*
 * UserInterface.cpp
 *
 *  Created on: Apr 21, 2025
 *      Author: emilr
 */

#include "UserInterface.h"
#include "sd1306/ssd1306.h"
#include "sd1306/ssd1306_fonts.h"

/* UserInterface::UserInterface(Status* status)
    : status(status), selected(false), selection(0),
      currentScreen(&UserInterface::drawWelcome),
      leftCall(&UserInterface::none),
      rightCall(&UserInterface::none),
      clickCall(&UserInterface::none) {
    setupWelcome();
} */


UserInterface::UserInterface(Status *status) {
    this->status = status;
    leftCall = &UserInterface::none;
    rightCall = &UserInterface::none;
    clickCall = &UserInterface::none;
    currentScreen = &UserInterface::none;

    selected = false;
    selection = 0;
}

void UserInterface::begin() {
    ssd1306_Init();
    char word[6] = "hello";
    char *buffer = word;

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_11x18, White);
    ssd1306_UpdateScreen();
}

void UserInterface::drawWelcome() {
}

void UserInterface::drawInfoScreen() {
}

void UserInterface::drawAutostart() {
}

void UserInterface::setupWelcome() {
}

void UserInterface::setupInfoScreen() {
}

void UserInterface::setupAutostart() {
}

void UserInterface::none() {
}

void UserInterface::select() {
}

void UserInterface::unselect() {
}

void UserInterface::increaseSelection() {
}

void UserInterface::decreaseSelection() {
}

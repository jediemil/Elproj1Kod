/*
 * UserInterface.cpp
 *
 *  Created on: Apr 21, 2025
 *      Author: emilr
 */

#include "UserInterface.h"
#include "sd1306/ssd1306.h"
#include "sd1306/ssd1306_fonts.h"
#include "include.h"
#include "main_cpp.h"
#include "stdio.h"

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
    /*char word[6] = "hello";
    char *buffer = word;

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_11x18, White);*/
    ssd1306_UpdateScreen();

    continueCall = &UserInterface::setupWelcome;
}

void UserInterface::drawWelcome() {
    ssd1306_Fill(Black);

    char line1[] = "Welcome\0";
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    char line2[] = "To InstaCooler\0";
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString(line2, Font_6x8, White);
    char line3[] = "Starting...\0";
    ssd1306_SetCursor(0, 19);
    ssd1306_WriteString(line3, Font_6x8, White);

    ssd1306_FillRectangle(4, 28, 124, 31, White);
    ssd1306_UpdateScreen();
}

void UserInterface::drawInfoScreen() {
    ssd1306_Fill(Black);

    char buffer[40];
    //buffer = std::format("Hello {}!", status->getMotorSpeed()*100);
    __disable_irq();
    sprintf(buffer, "Motor speed: %d%%", (int)(status->getMotorSpeed()*100.0 + 0.5));
    //sprintf(buffer, "Motor speed: d");
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_7x10, White);
    __enable_irq();

    __disable_irq();
    sprintf(buffer, "Time: %hu s / %hu s", status->getTimeLeft(), status->programLen);
    //sprintf(buffer, "Time: d s / d s");
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString(buffer, Font_7x10, White);
    __enable_irq();

    __disable_irq();
    float waterTemp = status->getWaterTemp();
    //Ligma
    if (-25.82 >= waterTemp || waterTemp >= 90.0) {
    	 sprintf(buffer, "Temp: N/A");
    } else {
        sprintf(buffer, "Temp: %.1f C", status->getWaterTemp());
    }
        //sprintf(buffer, "Time: d s / d s");
        ssd1306_SetCursor(0, 22);
        ssd1306_WriteString(buffer, Font_7x10, White);
        __enable_irq();

    float donePercentage = (float)(status->getTimeLeft()) / (float)(status->programLen);
    ssd1306_DrawRectangle(4, 38, 124, 50, White);
    ssd1306_FillRectangle(4, 38, 4+(int)(120*donePercentage), 50, White);


    if (selected) {
        clickCall = &UserInterface::runFunctionFromList;
        ssd1306_FillRectangle(20, 10, 108, 54, White);
        if (selection == 0) {
            ssd1306_Line(20, 21, 108, 21, Black);
        } else {
            ssd1306_Line(20, 31, 108, 31, Black);
        }

        char line1[] = "Cancel";
        ssd1306_SetCursor(22, 12);
        ssd1306_WriteString(line1, Font_6x8, Black);
        char line2[] = "Return";
        ssd1306_SetCursor(22, 22);
        ssd1306_WriteString(line2, Font_6x8, Black);

        //sprintf(buffer, "Val: %u", selection);
        //sprintf(buffer, "Val: u");
        //ssd1306_SetCursor(20, 26);
        //ssd1306_WriteString(buffer, Font_7x10, Black);
    } else if (clickCall != &UserInterface::none) {
        clickCall = &UserInterface::select;
    }

    ssd1306_UpdateScreen();
}

void UserInterface::drawAutostart() {
    ssd1306_Fill(Black);

    char line1[] = "Click to start\0";
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    char line2[] = "AUTOFUNCTIO\0";
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString(line2, Font_11x18, White);
    char line3[] = "N\0";
    ssd1306_SetCursor(56, 30);
    ssd1306_WriteString(line3, Font_16x26, White);

    ssd1306_UpdateScreen();
}

void UserInterface::drawLidOpen() {
    ssd1306_Fill(Black);

    char line1[] = "Lid Open\0";
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    char line1[] = "Close lid to continue\0";
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(line1, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void UserInterface::setupWelcome() {
	continueCall = &UserInterface::setupAutostart;

	currentScreen = &UserInterface::drawWelcome;
}

void UserInterface::setupInfoScreen() {
    maxSelection = 2;
    functionList = settingsPopupFuncs;

    clickCall = &UserInterface::select;
    leftCall = &UserInterface::decreaseSelection;
    rightCall = &UserInterface::increaseSelection;

    currentScreen = &UserInterface::drawInfoScreen;
}

void UserInterface::setupAutostart() {

    clickCall = &UserInterface::autostart;

    currentScreen = &UserInterface::drawAutostart;
}

void UserInterface::autostart() {
    status->programType = PROGRAM_TYPE_AUTO;
    status->programLen = 120;
    status->programRunning = true;

    setupInfoScreen();
}

void UserInterface::endProgram() {
    status->programRunning = false;
    selected = false;
    emptyCalls();

    continueCall = &UserInterface::setupAutostart;
}

void UserInterface::changeLidStatus() {
    if (status.lidOpen) {
        emptyCalls();
        currentScreen = &UserInterface::drawLidOpen;
    } else {
        setupAutostart();
    }
}

void UserInterface::emptyCalls() {
    leftCall = &UserInterface::none;
    rightCall = &UserInterface::none;
    clickCall = &UserInterface::none;
}

void UserInterface::none() {
}

void UserInterface::select() {
    selected = true;
}

void UserInterface::unselect() {
    selected = false;
}

void UserInterface::toggleSelect() {
    selected = !selected;
}

void UserInterface::increaseSelection() {
    selection++;
    selection %= maxSelection;
}

void UserInterface::decreaseSelection() {
    selection--;
    selection %= maxSelection;
}

void UserInterface::drawScreen() {
    (this->*currentScreen)();
}

void UserInterface::click() {
	(this->*clickCall)();
}

void UserInterface::rightSwipe() {
	(this->*rightCall)();
}

void UserInterface::leftSwipe() {
	(this->*leftCall)();
}
void UserInterface::continueEvent() {
    (this->*continueCall)();
}
void UserInterface::cancelEvent() {
    (this->*cancelCall)();
}

void UserInterface::runFunctionFromList() {
    (this->*functionList[selection])();
}

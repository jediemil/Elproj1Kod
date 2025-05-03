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
#include "bmp.h"
#include <algorithm>

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
    continueCall = &UserInterface::none;
    cancelCall = &UserInterface::none;

    selected = false;
    selection = 0;
    maxSelection = 2;

    selectedProgramLen = 120/5;
    selectedTemperature = 100;
    selectedDrinkSize = 33;
    selectedMotorSpeed = 10;
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
	ssd1306_DrawBitmap(0, 0, epd_bitmap_WhiteInstaCoolerLogo, 128, 64, White);

    ssd1306_FillRectangle(4, 59, 124, 63, White);

    ssd1306_UpdateScreen();
}

void UserInterface::drawInfoScreen() {
    ssd1306_Fill(Black);

    char buffer[40];
    //buffer = std::format("Hello {}!", status->getMotorSpeed()*100);
    __disable_irq();
    sprintf(buffer, "Motor speed: %d%%", (int)(status->getMotorSpeed()*100.0 + 0.5));
    __enable_irq();
    //sprintf(buffer, "Motor speed: d");
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_7x10, White);

    __disable_irq();
    sprintf(buffer, "Time: %hus / %hus", status->getTimeLeft(), status->programLen);
    __enable_irq();
    //sprintf(buffer, "Time: d s / d s");
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString(buffer, Font_7x10, White);

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
    ssd1306_SetCursor(20, 20);
    ssd1306_WriteString(line1, Font_11x18, White);
    char line2[] = "Close the lid\0";
    char line3[] = "to continue\0";
    ssd1306_SetCursor(0, 39);
    ssd1306_WriteString(line2, Font_7x10, White);
    ssd1306_SetCursor(0, 51);
    ssd1306_WriteString(line3, Font_7x10, White);

    ssd1306_UpdateScreen();
}

void UserInterface::drawCustomProgram() {
	ssd1306_Fill(Black);

	char line1[] = "Click to start\0";
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(line1, Font_7x10, White);
	char line2[] = "CUSTOM PROGRA\0";
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(line2, Font_11x18, White);
	char line3[] = "M\0";
	ssd1306_SetCursor(56, 30);
	ssd1306_WriteString(line3, Font_16x26, White);

	ssd1306_UpdateScreen();
}

void UserInterface::drawCredits() {
	ssd1306_Fill(Black);
	char line1[] = "InstaCooler\0";
	char line2[] = "Erik Nydahl\0";
	char line3[] = "Charlie Sandvall\0";
	char line4[] = "Jorm Akerberg\0";
	char line5[] = "Karl Stenberg\0";
	char line6[] = "Emil Reyier Hedin\0";

	ssd1306_SetCursor(3, 0);
	ssd1306_WriteString(line1, Font_11x18, White);
	ssd1306_SetCursor(13, 19);
	ssd1306_WriteString(line2, Font_6x8, White);
	ssd1306_SetCursor(2, 28);
	ssd1306_WriteString(line3, Font_6x8, White);
	ssd1306_SetCursor(4, 37);
	ssd1306_WriteString(line4, Font_6x8, White);
	ssd1306_SetCursor(2, 46);
	ssd1306_WriteString(line5, Font_6x8, White);
	ssd1306_SetCursor(7, 55);
	ssd1306_WriteString(line6, Font_6x8, White);

	ssd1306_UpdateScreen();
}

void UserInterface::drawCustomProgramSettings() {
	ssd1306_Fill(Black);
	selection = (selection + 6) % 6;

	selectedProgramLen = std::max(std::min((int)selectedProgramLen, 60), 6);
	selectedTemperature = std::max(std::min((int)selectedTemperature, 200), 60);
	selectedDrinkSize = std::max(std::min((int)selectedDrinkSize, 100), 7);
	selectedMotorSpeed = std::max(std::min((int)selectedMotorSpeed, 50), 4);

	if (selected) {
		clickCall = &UserInterface::unselect;
		if (selection <= 3) {
			leftCall = &UserInterface::decreaseVariable;
			rightCall = &UserInterface::increaseVariable;
		} else if (selection == 4) {
			selected = false;
			selection = 0;
			startProgram();
		} else if (selection == 5) {
			selected = false;
			selection = 0;
			setupCustomProgram();
		}
	} else {
		clickCall = &UserInterface::select;
		leftCall = &UserInterface::decreaseSelection;
		rightCall = &UserInterface::increaseSelection;
	}

	char line1[] = "Custom settings\0";
	char line2[] = "Time\0";
	char line3[] = "Goal temp\0";
	char line4[] = "Drink size\0";
	char line5[] = "Motor speed\0";
	char line6[] = "Start Program\0";
	char line7[] = "Cancel\0";

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(line1, Font_6x8, White);
	/*ssd1306_SetCursor(0, 9);
	ssd1306_WriteString(line2, Font_6x8, White);
	ssd1306_SetCursor(0, 18);
	ssd1306_WriteString(line3, Font_6x8, White);
	ssd1306_SetCursor(0, 27);
	ssd1306_WriteString(line4, Font_6x8, White);
	ssd1306_SetCursor(0, 36);
	ssd1306_WriteString(line5, Font_6x8, White);
	ssd1306_SetCursor(0, 45);
	ssd1306_WriteString(line6, Font_6x8, White);
	ssd1306_SetCursor(0, 54);
	ssd1306_WriteString(line7, Font_6x8, White);*/

	char buffer[10];
	__disable_irq();
	sprintf(buffer, "%d S", selectedProgramLen*5);
	__enable_irq();
	drawSettingCell(9, (selection==0)&&!selected, (selection==0)&&selected, line2, buffer);

	__disable_irq();
	sprintf(buffer, "%.1f C", selectedTemperature/10.0);
	__enable_irq();
	drawSettingCell(18, (selection==1)&&!selected, (selection==1)&&selected, line3, buffer);

	__disable_irq();
	sprintf(buffer, "%d cL", selectedDrinkSize);
	__enable_irq();
	drawSettingCell(27, (selection==2)&&!selected, (selection==2)&&selected, line4, buffer);

	__disable_irq();
	sprintf(buffer, "%d %%", selectedMotorSpeed);
	__enable_irq();
	drawSettingCell(36, (selection==3)&&!selected, (selection==3)&&selected, line5, buffer);
	drawSettingCell(45, (selection==4)&&!selected, (selection==4)&&selected, line6, NULL);
	drawSettingCell(54, (selection==5)&&!selected, (selection==5)&&selected, line7, NULL);

	ssd1306_UpdateScreen();
}

void UserInterface::drawSettingCell(uint8_t y, bool highlighted, bool selected, char* text, char* value) {
	if (highlighted) {
	    ssd1306_FillRectangle(0, y, 100, y+8, White);
	}
	ssd1306_SetCursor(0, y);
	ssd1306_WriteString(text, Font_6x8, (SSD1306_COLOR)!highlighted);

	if (value == NULL) return;

	if (selected) {
		    ssd1306_FillRectangle(101, y, 128, y+8, White);
	}
	ssd1306_SetCursor(101, y);
	ssd1306_WriteString(value, Font_6x8, (SSD1306_COLOR)!selected);
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

    selected = false;
    selection = 0;

    currentScreen = &UserInterface::drawInfoScreen;
}

void UserInterface::setupAutostart() {

    clickCall = &UserInterface::autostart;
    leftCall = &UserInterface::setupCredits;
    rightCall = &UserInterface::setupCustomProgram;
    currentScreen = &UserInterface::drawAutostart;
}

void UserInterface::setupCredits() {
    emptyCalls();
    leftCall = &UserInterface::setupCustomProgram;
    rightCall = &UserInterface::setupAutostart;
    currentScreen = &UserInterface::drawCredits;
}

void UserInterface::setupCustomProgram() {
    emptyCalls();
    leftCall = &UserInterface::setupAutostart;
    rightCall = &UserInterface::setupCredits;
    clickCall = &UserInterface::setupCustomProgramSettings;
    currentScreen = &UserInterface::drawCustomProgram;
}

void UserInterface::setupCustomProgramSettings() {
    emptyCalls();
    clickCall = &UserInterface::select;
    leftCall = &UserInterface::decreaseSelection;
    rightCall = &UserInterface::increaseSelection;
    selected = false;
    selection = 0;
    maxSelection = 6;
    currentScreen = &UserInterface::drawCustomProgramSettings;
}

void UserInterface::autostart() {
    status->programType = PROGRAM_TYPE_AUTO;
    status->programLen = 120;
    status->programRunning = true;

    setupInfoScreen();
}

void UserInterface::startProgram() {
	status->programType = PROGRAM_TYPE_CUSTOM;
	status->programLen = selectedProgramLen*5;
	status->selectedMotorSpeed = selectedMotorSpeed/100.0;
	status->targetTemp = selectedTemperature/10.0;
	status->drinkSize = selectedDrinkSize*10; //cL -> mL
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
    if (status->lidOpen) {
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
    selection = selection - 1 + maxSelection;
    selection %= maxSelection;
}

void UserInterface::increaseVariable() {
	(*variableSelection[selection])++;
}

void UserInterface::decreaseVariable() {
	(*variableSelection[selection])--;
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

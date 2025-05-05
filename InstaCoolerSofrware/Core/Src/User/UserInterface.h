/*
 * UserInterface.h
 *
 *  Created on: Apr 21, 2025
 *      Author: emilr
 */

#ifndef SRC_USER_USERINTERFACE_H_
#define SRC_USER_USERINTERFACE_H_

#include "Status.h"

class UserInterface {
public:
    typedef void (UserInterface::*method_function)();

    UserInterface(Status *status);

    void begin();

    void leftSwipe();
    void rightSwipe();
    void click();

    void cancelEvent();
    void continueEvent();
    void changeLidStatus();

    void drawScreen();

    void setupWelcome();
    void setupAutostart();

private:
    void drawWelcome();
    void drawInfoScreen();
    void drawAutostart();
    void drawLidOpen();
    void drawCustomProgram();
    void drawCredits();
    void drawCustomProgramSettings();
    void drawCreditsSelection();

    void setupInfoScreen();
    void setupCustomProgram();
    void setupCredits();
    void setupCustomProgramSettings();
    void setupCreditsSelection();

    void none();
    void select();
    void unselect();
    void toggleSelect();
    void increaseSelection();
    void decreaseSelection();
    void increaseVariable();
    void decreaseVariable();

    void runFunctionFromList();
    void emptyCalls();

    void drawSettingCell(uint8_t y, bool highlighted, bool selected, char* text, char* value);

    bool selected;
    uint16_t selection;
    uint16_t maxSelection;

    uint16_t selectedProgramLen;
    uint16_t selectedTemperature;
    uint16_t selectedDrinkSize;
    uint16_t selectedMotorSpeed;

    uint16_t* variableSelection[2] = {&selectedProgramLen, &selectedMotorSpeed};//&selectedTemperature, &selectedDrinkSize, &selectedMotorSpeed};

    Status *status;

    void autostart();
    void startProgram();
    void endProgram();

    void (UserInterface::*leftCall)();
    void (UserInterface::*rightCall)();
    void (UserInterface::*clickCall)();
    void (UserInterface::*currentScreen)();

    void (UserInterface::*cancelCall)();
    void (UserInterface::*continueCall)();

    //method_function screens[3] = {&UserInterface::drawWelcome, &UserInterface::drawInfoScreen, &UserInterface::drawAutostart};
    //method_function setups[3] = {&UserInterface::setupWelcome, &UserInterface::setupInfoScreen, &UserInterface::setupAutostart};
    method_function *functionList;
    method_function settingsPopupFuncs[2] = {&UserInterface::endProgram, &UserInterface::unselect};
};

#endif /* SRC_USER_USERINTERFACE_H_ */

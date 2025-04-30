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

private:
    void drawWelcome();
    void drawInfoScreen();
    void drawAutostart();
    void drawLidOpen();

    void setupInfoScreen();
    void setupAutostart();

    void none();
    void select();
    void unselect();
    void toggleSelect();
    void increaseSelection();
    void decreaseSelection();

    void runFunctionFromList();
    void emptyCalls();

    bool selected;
    uint32_t selection;
    uint32_t maxSelection;

    Status *status;

    void autostart();
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

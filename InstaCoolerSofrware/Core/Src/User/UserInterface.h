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

    void drawScreen();

private:
    void drawWelcome();
    void drawInfoScreen();
    void drawAutostart();

    void setupWelcome();
    void setupInfoScreen();
    void setupAutostart();

    void none();
    void select();
    void unselect();
    void toggleSelect();
    void increaseSelection();
    void decreaseSelection();

    bool selected;
    int selection;

    Status *status;

    void (UserInterface::*leftCall)();
    void (UserInterface::*rightCall)();
    void (UserInterface::*clickCall)();
    void (UserInterface::*currentScreen)();

    //method_function screens[3] = {&UserInterface::drawWelcome, &UserInterface::drawInfoScreen, &UserInterface::drawAutostart};
    //method_function setups[3] = {&UserInterface::setupWelcome, &UserInterface::setupInfoScreen, &UserInterface::setupAutostart};
};

#endif /* SRC_USER_USERINTERFACE_H_ */

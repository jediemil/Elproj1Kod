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

	UserInterface(Status* status);

    void leftSwipe();
    void rightSwipe();
    void click();

    void drawScreen();

    void setScreen(method_function screen_function);

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
    void increaseSelection();
    void decreaseSelection();

   	Status* status;

    method_function leftCall;
	method_function rightCall;
	method_function clickCall;
    method_function currentScreen;
	//method_function screens[3] = {&UserInterface::drawWelcome, &UserInterface::drawInfoScreen, &UserInterface::drawAutostart};
	//method_function setups[3] = {&UserInterface::setupWelcome, &UserInterface::setupInfoScreen, &UserInterface::setupAutostart};
};

#endif /* SRC_USER_USERINTERFACE_H_ */

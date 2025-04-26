/*
 * UserInterface.cpp
 *
 *  Created on: Apr 21, 2025
 *      Author: emilr
 */

#include "UserInterface.h"

UserInterface::UserInterface(Status* status) {
    leftCall = &UserInterface::none;
    rightCall = &UserInterface::none;
    clickCall = &UserInterface::none;
    currentScreen = &UserInterface::drawWelcome;

    this->status = status;
}

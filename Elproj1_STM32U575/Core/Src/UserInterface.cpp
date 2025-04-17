/*
 * UserInterface.cpp
 *
 *  Created on: Apr 17, 2025
 *      Author: Carl Sandvall
 */

#include "UserInterface.h"
#include <stdio.h>

UserInterface::UserInterface() {


}

UserInterface::~UserInterface() {
	// TODO Auto-generated destructor stub
}

void UserInterface::begin() {
	char buffer[20];

	ssd1306_Fill(Black);

	//sprintf(buffer, "Count: %d", encoder_counter);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(buffer, Font_11x18, White);
	ssd1306_UpdateScreen();
}

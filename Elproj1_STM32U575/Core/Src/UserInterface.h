/*
 * UserInteface.h
 *
 *  Created on: Apr 17, 2025
 *      Author: Carl Sandvall
 */

#ifndef SRC_USERINTERFACE_H_
#define SRC_USERINTERFACE_H_

#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"

class UserInterface {
public:
	UserInterface();
	virtual ~UserInterface();

	void begin();

	void rightTurn();
	void leftTurn();
	void click();
	void reset();

private:
	void drawToScreen();
};

#endif /* SRC_USERINTERFACE_H_ */

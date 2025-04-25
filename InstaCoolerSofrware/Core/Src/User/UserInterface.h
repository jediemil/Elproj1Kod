/*
 * UserInterface.h
 *
 *  Created on: Apr 21, 2025
 *      Author: emilr
 */

#ifndef SRC_USER_USERINTERFACE_H_
#define SRC_USER_USERINTERFACE_H_

class UserInterface {
public:
	UserInterface();
	virtual ~UserInterface();

private:
	void drawWelcome();
	void drawInfoScreen();
	void drawAutostart();
};

#endif /* SRC_USER_USERINTERFACE_H_ */

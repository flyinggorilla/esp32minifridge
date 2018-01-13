/*
 * MusicPlayer.h
 *
 *  Created on: 10.04.2017
 *      Author: bernd
 */

#ifndef MAIN_FRIDGECONTROLLER_H_
#define MAIN_FRIDGECONTROLLER_H_

#include "String.h"

class FridgeController {
public:
	FridgeController();
	virtual ~FridgeController();

	/*
	 * @brief initializes the GPIO & PWM subsystem
	 */
	void init();

	void FanHot(bool onoff);



private:
	//SemaphoreHandle_t playerMutex;
};

#endif /* MAIN_FRIDGECONTROLLER_H_ */

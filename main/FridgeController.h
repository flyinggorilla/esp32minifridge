/*
 * MusicPlayer.h
 *
 *  Created on: 10.04.2017
 *      Author: bernd
 */

#ifndef MAIN_FRIDGECONTROLLER_H_
#define MAIN_FRIDGECONTROLLER_H_

#include "String.h"
#include "owb.h"


class FridgeController
{
  public:
	FridgeController();
	virtual ~FridgeController();

	/*
	 * @brief initializes the GPIO & PWM subsystem
	 */
	void init(bool power, float targetTemperature);
	bool SetTargetTemperature(float targetTemperature);
	float GetTargetTemperature() { return mfTargetTemperature; };
	float GetActualTemperature() { return mfActualTemperature; };
	bool SetDeadBand(float deadBand);
	float GetDeadBand() { return mfDeadBand; };

	/*
	* @return true if ok, false if there was an error during the measurement
	*/
	bool MeasureActualTemperature();
	void Power(bool onoff);
	bool GetPower() { return mbIsPower; };
	void Run();

  private:
	//SemaphoreHandle_t playerMutex;
	float mfTargetTemperature = 20.0;
	float mfActualTemperature = 0;
	bool mbIsPower = true;
	bool mbIsError = false;
	bool mbIsPeltier = false;
	float mfDeadBand = 4.0;

	void Fan(bool onoff);
	void Led(uint8_t brightness);
	void Peltier(bool onoff);

	OneWireBus *mOwb = NULL;

};

#endif /* MAIN_FRIDGECONTROLLER_H_ */

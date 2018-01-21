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
#include "ds18b20.h"


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
	bool IsPower() { return mbIsPower; };
	void Run();
	bool IsError() { return mbIsError; };
	bool IsCooling() { return mbIsPeltier; };

  private:
	//SemaphoreHandle_t playerMutex;
	float mfTargetTemperature = 20.0;
	volatile float mfActualTemperature = 0;
	bool mbIsPower = true;
	volatile bool mbIsError = false;
	volatile bool mbIsPeltier = false;
	float mfDeadBand = 4.0;

	void Fan(bool onoff);
	void Peltier(bool onoff);

	OneWireBus *mOwb = NULL;
	DS18B20_Info mDs18b20;
	owb_rmt_driver_info mRmtDriverInfo;

};

#endif /* MAIN_FRIDGECONTROLLER_H_ */

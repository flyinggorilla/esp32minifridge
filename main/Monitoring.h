//&#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "String.h"
#include "WebClient.h"

#ifndef MAIN_MONITORING_H_
#define MAIN_MONITORING_H_

struct TMeasurement
{
  float temperature;
  float targettemp;
  bool cooling;
  bool power;
  uint64_t timestamp; // UTC epoch, milliseconds
};

/*
 * @brief 
 */
class Monitoring
{
  public:
	Monitoring();
	virtual ~Monitoring();

	/*
	 * @brief  Starts a background thread that sends monitoring data to Dynatrace every minute.
	 *
	 */
	void Start(String environment, String apitoken, String deviceidentifier);

	
	bool Add(TMeasurement &measurement);

	/*
	* @brief do not call. internal used.
	*/
	void RunInternal();


  private:
	bool Peek();
	unsigned int Length();
	bool Remove(TMeasurement &measurement);

	bool RegisterCustomMetric(String metric, String data);
	bool DeleteCustomMetric(String metric);

	QueueHandle_t mhQueue = 0;
	WebClient mWebClient;


	String msEnvironment;
	String msAuthorizationHeader;
	String msDeviceIdentifier;
};

#endif /* MAIN_MONITORING_H_ */

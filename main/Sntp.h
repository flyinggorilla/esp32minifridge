#include <stdint.h>
#include <stdio.h>
#include "String.h"

#ifndef MAIN_SNTP_H_
#define MAIN_SNTP_H_


/*
 * @brief 
 */
class Sntp {
public:
	Sntp();
	virtual ~Sntp();

	/*
	 * @brief  Initializes SNTP. Default auto-refresh is 60 minutes. 
	 * 		 Note: this function should be called after WIFI has been initialized properly
	 *
	 * @return false on SNTP initialization error
	 *
	 */
	bool Init(String ntpServer);

	

	/*
	 * @brief   Unix Epoch time in UTC milliseconds
	 * @return  0 when NTP has not provided a time in the range of 2018 .. 2127
	 */
	uint64_t GetEpochMillisecondsUTC();

private:
	String msSntpServer;
};

#endif /* MAIN_SNTP_H_ */



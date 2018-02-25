#include "Sntp.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "apps/sntp/sntp.h"

static const char *LOGTAG = "sntp";

Sntp::Sntp()
{
}

Sntp::~Sntp()
{
}

bool Sntp::Init(String sntpServer)
{
  ESP_LOGI(LOGTAG, "Initializing SNTP");
  msSntpServer = sntpServer;
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, (char*)msSntpServer.c_str());
  sntp_init();
  return true;
}

uint64_t Sntp::GetEpochMillisecondsUTC() {
  timeval t;
  gettimeofday(&t, NULL);

  // dates in the range 2018 ... 2128 are treated as valid
  if (t.tv_usec < 1500000000000000U || t.tv_usec > 5000000000000000U) {
    return 0;
  }

  return t.tv_usec / 1000;
}

#include "Monitoring.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "cJSON.h"
#include "Wifi.h"

static const char *LOGTAG = "monitoring";
static const char *HEADER_CONTENTTYPEJSON = "Content-Type: application/json";

extern Wifi wifi;

Monitoring::Monitoring()
{
}

Monitoring::~Monitoring()
{
}

bool Monitoring::Add(TMeasurement &measurement)
{
  if (pdTRUE == xQueueSend(mhQueue, &measurement, 0)) // never block/wait
    return true;
  ESP_LOGW(LOGTAG, "Monitoring Queue Full");
  return false;
}

bool Monitoring::Peek()
{
  //	vTaskDelay(100 / portTICK_PERIOD_MS);
  TMeasurement measurement;
  return pdTRUE == xQueuePeek(mhQueue, &measurement, 10 * 1000 / portTICK_PERIOD_MS); // wait 10 seconds
}

unsigned int Monitoring::Length()
{
  return uxQueueMessagesWaiting(mhQueue);
}

bool Monitoring::Remove(TMeasurement &measurement)
{
  return pdTRUE == xQueueReceive(mhQueue, &measurement, 0 / portTICK_PERIOD_MS); // wait 0 seconds
}
/*
		"series" : [
		  {
			"timeseriesId" : "custom:minifridge.temperature",
			
			"dataPoints" : [ [ ` + timestampString + `, ` + strconv.FormatFloat(float64(temperature), 'f', 2, 32) + ` ] ]
		  },
*/

void task_function_run(void *user_data)
{
  ((Monitoring *)((const char *)user_data))->RunInternal();
}

void Monitoring::Start(String environment, String apitoken, String deviceidentifier)
{
  msEnvironment = environment;
  msAuthorizationHeader = "Authorization: Api-Token " + apitoken;
  msDeviceIdentifier = deviceidentifier;
  mhQueue = xQueueCreate(90, sizeof(TMeasurement));
  // Pin firmware update task to core 0 --- otherwise we get weird crashes
  xTaskCreatePinnedToCore(&task_function_run, "monitoringsender", 8192, (void *)this, 6, NULL, 1);
}

bool Monitoring::DeleteCustomMetric(String metric)
{
  String surl;
  surl.printf("https://%s.live.dynatrace.com/api/v1/timeseries/custom:%s/", msEnvironment.c_str(), metric.c_str());
  Url url;
  url.Parse(surl);

  mWebClient.Prepare(&url);
  mWebClient.AddHttpHeader(msAuthorizationHeader);
  mWebClient.AddHttpHeaderCStr(HEADER_CONTENTTYPEJSON);
  unsigned short httpStatus = mWebClient.HttpDelete();
  ESP_LOGI(LOGTAG, "  Request %s", url.GetUrl().c_str());
  ESP_LOGI(LOGTAG, "  Response %s", mWebClient.GetResponseData().c_str());
  if (httpStatus < 200 || httpStatus > 299)
  {
    ESP_LOGE(LOGTAG, "Error deleting metric - HTTP Status %u", httpStatus);
    return false;
  }
  return true;
}

bool Monitoring::RegisterCustomMetric(String metric, String data)
{
  String surl;
  surl.printf("https://%s.live.dynatrace.com/api/v1/timeseries/custom:%s/", msEnvironment.c_str(), metric.c_str());
  Url url;
  url.Parse(surl);

  mWebClient.Prepare(&url);
  mWebClient.AddHttpHeader(msAuthorizationHeader);
  mWebClient.AddHttpHeaderCStr(HEADER_CONTENTTYPEJSON);
  unsigned short httpStatus = mWebClient.HttpPut(data);
  ESP_LOGI(LOGTAG, "  Request %s\r\n%s", url.GetUrl().c_str(), data.c_str());
  ESP_LOGI(LOGTAG, "  Response %s", mWebClient.GetResponseData().c_str());
  if (httpStatus < 200 || httpStatus > 299)
  {
    ESP_LOGE(LOGTAG, "Error registering metric - HTTP Status %u", httpStatus);
    return false;
  }
  return true;
}


void Monitoring::RunInternal()
{
  ESP_LOGI(LOGTAG, "Starting Monitoring with Dynatrace ....");

  //SendTimeseriesData();

  
  /*ESP_LOGI(LOGTAG, "Delete Metrics with Dynatrace ....");
  DeleteCustomMetric("minifridge.temperature");
  DeleteCustomMetric("minifridge.temperature.target");
  DeleteCustomMetric("minifridge.power");
  DeleteCustomMetric("minifridge.active");
  DeleteCustomMetric("minifridge.cooling"); 

  ESP_LOGI(LOGTAG, "Register Metrics with Dynatrace ....");
  RegisterCustomMetric("minifridge.temperature", "{\"displayName\":\"Temperature\",\"unit\":\"Count\",\"dimensions\":[],\"types\":[\"MiniFridge\"]}");
  RegisterCustomMetric("minifridge.temperature.target", "{\"displayName\":\"Target Temperature\",\"unit\":\"Count\",\"dimensions\":[],\"types\":[\"MiniFridge\"]}");
  RegisterCustomMetric("minifridge.power", "{\"displayName\":\"Power\",\"unit\":\"Ratio\",\"dimensions\":[],\"types\":[\"MiniFridge\"]}");
  RegisterCustomMetric("minifridge.cooling", "{\"displayName\":\"Cooling\",\"unit\":\"Percent\",\"dimensions\":[],\"types\":[\"MiniFridge\"]}");
*/
  while (true)
  {
    if (Peek())
    {

      String ipaddress = wifi.GetLocalAddress();
      String configUrl = "http://" + ipaddress + ":80";
      String timestamp;

      cJSON *root, *ipaddresses, *ports, *tags, *properties, *series, *seriesitem;
      cJSON *dataPointsTemperature, *dataPointsTargettemp, *dataPointsCooling, *dataPointsPower;
      root = cJSON_CreateObject();
      cJSON_AddStringToObject(root, "displayName", msDeviceIdentifier.c_str());
      cJSON_AddItemToObject(root, "ipAddresses", ipaddresses = cJSON_CreateArray());
      cJSON_AddItemToArray(ipaddresses, cJSON_CreateString(ipaddress.c_str()));
      cJSON_AddItemToObject(root, "listenPorts", ports = cJSON_CreateArray());
      cJSON_AddItemToArray(ports, cJSON_CreateString("80"));
      cJSON_AddStringToObject(root, "type", "MiniFridge");
      cJSON_AddStringToObject(root, "favicon", "http://icons.iconarchive.com/icons/iconka/saint-whiskers/256/cat-fridge-icon.png");
      cJSON_AddStringToObject(root, "configUrl", configUrl.c_str());
      cJSON_AddItemToObject(root, "tags", tags = cJSON_CreateArray());
      cJSON_AddItemToObject(root, "properties", properties = cJSON_CreateObject());
      cJSON_AddStringToObject(properties, "ESP-IDF version", esp_get_idf_version());
      cJSON_AddItemToObject(root, "series", series = cJSON_CreateArray());

      cJSON_AddItemToArray(series, seriesitem = cJSON_CreateObject());
      cJSON_AddStringToObject(seriesitem, "timeseriesId", "custom:minifridge.temperature"); //***********
      cJSON_AddItemToObject(seriesitem, "dataPoints", dataPointsTemperature = cJSON_CreateArray());

      cJSON_AddItemToArray(series, seriesitem = cJSON_CreateObject());
      cJSON_AddStringToObject(seriesitem, "timeseriesId", "custom:minifridge.temperature.target"); //***********
      cJSON_AddItemToObject(seriesitem, "dataPoints", dataPointsTargettemp = cJSON_CreateArray());

      cJSON_AddItemToArray(series, seriesitem = cJSON_CreateObject());
      cJSON_AddStringToObject(seriesitem, "timeseriesId", "custom:minifridge.cooling"); //***********
      cJSON_AddItemToObject(seriesitem, "dataPoints", dataPointsCooling = cJSON_CreateArray());

      cJSON_AddItemToArray(series, seriesitem = cJSON_CreateObject());
      cJSON_AddStringToObject(seriesitem, "timeseriesId", "custom:minifridge.power"); //***********
      cJSON_AddItemToObject(seriesitem, "dataPoints", dataPointsPower = cJSON_CreateArray());

      TMeasurement measurement;
      while (Remove(measurement))
      {
        cJSON *entries;
        cJSON_AddItemToArray(dataPointsTemperature, entries = cJSON_CreateArray());
        timestamp = "";
        timestamp.printf("%llu", measurement.timestamp);

        cJSON_AddItemToArray(entries, cJSON_CreateRaw(timestamp.c_str()));
        cJSON_AddItemToArray(entries, cJSON_CreateNumber(measurement.temperature));

        cJSON_AddItemToArray(dataPointsTargettemp, entries = cJSON_CreateArray());
        cJSON_AddItemToArray(entries, cJSON_CreateRaw(timestamp.c_str()));
        cJSON_AddItemToArray(entries, cJSON_CreateNumber(measurement.targettemp));

        cJSON_AddItemToArray(dataPointsCooling, entries = cJSON_CreateArray());
        cJSON_AddItemToArray(entries, cJSON_CreateRaw(timestamp.c_str()));
        cJSON_AddItemToArray(entries, cJSON_CreateNumber(measurement.cooling ? 100 : 0));

        cJSON_AddItemToArray(dataPointsPower, entries = cJSON_CreateArray());
        cJSON_AddItemToArray(entries, cJSON_CreateRaw(timestamp.c_str()));
        cJSON_AddItemToArray(entries, cJSON_CreateNumber(measurement.power ? 1 : 0));
      }

      char *json = cJSON_Print(root);

      ESP_LOGI(LOGTAG, "JSON: <<< %s >>>", json);

      String surl = "https://";
      surl += msEnvironment;
      surl += ".live.dynatrace.com/api/v1/entity/infrastructure/custom/";
      surl += msDeviceIdentifier;
      Url url;
      url.Parse(surl);

      mWebClient.Prepare(&url);
      mWebClient.AddHttpHeader(msAuthorizationHeader);
      mWebClient.AddHttpHeaderCStr(HEADER_CONTENTTYPEJSON);
      unsigned short httpStatus = mWebClient.HttpPost(json, strlen(json));
      ESP_LOGI(LOGTAG, "  Request %s\r\n%s", url.GetUrl().c_str(), json);
      ESP_LOGI(LOGTAG, "  Response %s", mWebClient.GetResponseData().c_str());
      if (httpStatus < 200 || httpStatus > 299)
      {
        ESP_LOGE(LOGTAG, "Error sending data - HTTP Status %u", httpStatus);
      }

      cJSON_Delete(root);
    }

    mWebClient.Clear();
    vTaskDelay(60 * 1000 / portTICK_PERIOD_MS); // 60 sec delay
  }
}

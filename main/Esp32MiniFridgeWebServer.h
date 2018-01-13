#ifndef MAIN_Esp32MiniFridgeWebServer_H_
#define MAIN_Esp32MiniFridgeWebServer_H_

#include "ota.h"
#include "freertos/FreeRTOS.h"
#include "openssl/ssl.h"
#include "WebServer.h"

class DisplayCharter;

class Esp32MiniFridgeWebServer : public WebServer{
public:
	Esp32MiniFridgeWebServer();
	virtual ~Esp32MiniFridgeWebServer();

	bool StartWebServer();

	virtual bool HandleRequest(HttpRequestParser& httpParser, HttpResponse& httpResponse);

private:
	bool mbRestart;

	Ota mOta;

};

#endif 

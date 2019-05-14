#pragma once

#include <iostream>
#include <QObject>
#include <coap2/coap.h>
#include <netdb.h>

class TrikCoapClient : public QObject {
	Q_OBJECT

public:
	TrikCoapClient();
	~TrikCoapClient();
	static int resolve_address(const char *host, const char *service, coap_address_t *dst);
	Q_INVOKABLE void setPowerMotor(std::string name, int power);
	Q_INVOKABLE void start(void);

private:
	coap_context_t *ctx = nullptr;
	coap_address_t dst;
	coap_endpoint_t *endpoint = nullptr;
	coap_session_t *session = nullptr;
};


#pragma once

#include <iostream>
#include <QObject>
#include <coap2/coap.h>
#include <netdb.h>

class TrikCoapClient : public QObject {
	Q_OBJECT

public:
	virtual ~TrikCoapClient() {};
	static int resolve_address(const char *host, const char *service, coap_address_t *dst);
	
public slots:
	void setPowerMotor(QString name, int power);
	void start();
	
private:
	coap_context_t *ctx;
	coap_address_t dst;
	coap_endpoint_t *endpoint;
	coap_session_t *session;
};


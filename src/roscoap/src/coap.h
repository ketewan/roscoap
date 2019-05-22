#pragma once

#include <iostream>
#include <QObject>
#include <QTimer>
#include <coap2/coap.h>
#include <netdb.h>
#include <QVector>
#include <QString>

class TrikCoapClient : public QObject {
	Q_OBJECT

public:
	TrikCoapClient();
	~TrikCoapClient();
	static int resolve_address(const char *host, const char *service, coap_address_t *dst);
	static TrikCoapClient* instance;

public slots:
	void setPowerMotor(QString name, int power);
	void subscribeForSensor(QString name);
	void changeLed(QString color);
	void start();
	void coapRunOnce();
	
signals:
	void newData(QString sensor, int data);

private:
	void emitSignal(QString sensor, int data);
	QTimer *timer;
	int token = 0;
	coap_context_t *ctx;
	coap_address_t dst;
	coap_endpoint_t *endpoint;
	coap_session_t *session;

	coap_pdu_t * coap_new_request(coap_context_t *context, 
				 coap_session_t *session, 
				 char request_code,
				 unsigned char *data,
				 size_t length,
				 coap_binary_t token);
};


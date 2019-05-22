#include <QApplication>
#include "coap.h"

TrikCoapClient* TrikCoapClient::instance = nullptr;

int TrikCoapClient::resolve_address(const char *host, const char *service, coap_address_t *dst) {
	struct addrinfo *res, *ainfo;
	struct addrinfo hints;
	int error, len = -1;

	memset(&hints, 0, sizeof(hints));
	memset(dst, 0, sizeof(*dst));
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_family = AF_UNSPEC;

	error = getaddrinfo(host, service, &hints, &res);

	if (error != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
		return error;
	}

	for (ainfo = res; ainfo != NULL; ainfo = ainfo->ai_next) {
		switch (ainfo->ai_family) {
		case AF_INET6:
		case AF_INET:
			len = dst->size = ainfo->ai_addrlen;
			memcpy(&dst->addr.sin6, ainfo->ai_addr, dst->size);
		default:
			;
		}
	}
	freeaddrinfo(res);
	return len;
}

TrikCoapClient::TrikCoapClient() {
	TrikCoapClient::instance = this;
}

TrikCoapClient::~TrikCoapClient() {
  coap_session_release(session);
  coap_free_context(ctx);
  coap_cleanup();
}

coap_pdu_t * TrikCoapClient::coap_new_request(coap_context_t *context, 
				 coap_session_t *session, 
				 char request_code,
				 unsigned char *data,
				 size_t length,
				 coap_binary_t token) {
  coap_pdu_t *pdu;

  pdu = coap_pdu_init(COAP_MESSAGE_CON, request_code, coap_new_message_id(session),
                      coap_session_max_pdu_size(session));

  if (!pdu) {
    coap_log(LOG_DEBUG, "failed to init pdu\n");
    return NULL;
  }

  if (!coap_add_token(pdu, token.length, token.s)) {
    coap_log(LOG_DEBUG, "cannot add token to request\n");
    return NULL;
  }

  if (data && length) {
    if (!coap_add_data(pdu, length, data)) {
	  coap_log(LOG_DEBUG, "failed to add data\n");
      return NULL;
  	}
  }

  return pdu;
}

void TrikCoapClient::setPowerMotor(QString name, int power) {
	auto pdu = coap_new_request(ctx, session, COAP_REQUEST_GET, NULL, 0, {0, (unsigned char *)"hello"});	
  	
	coap_add_option(pdu, COAP_OPTION_URI_PATH, 6, 
									(unsigned char *)"motors");
	
	auto query = name.toStdString() + "=" + std::to_string(power);
	coap_add_option(pdu, COAP_OPTION_URI_QUERY, query.length(), 
								(unsigned char *)query.c_str());
	
  	coap_send(session, pdu);
}

void TrikCoapClient::changeLed(QString color) {
	auto pdu = coap_new_request(ctx, session, COAP_REQUEST_GET, NULL, 0, {0, (unsigned char *)"0"});
	coap_add_option(pdu, COAP_OPTION_URI_PATH, 3, 
									(unsigned char *)"led");
	auto query = color.toStdString();
	coap_add_option(pdu, COAP_OPTION_URI_QUERY, query.length(), 
								(unsigned char *)query.c_str());
	
  	coap_send(session, pdu);
}

void TrikCoapClient::subscribeForSensor(QString name) {
	std::cout << name.toStdString() << std::endl;
	auto msg = (unsigned char *)name.toStdString().c_str();
	coap_binary_t token = {sizeof(msg), msg};
	auto pdu = coap_new_request(ctx, session, COAP_REQUEST_GET, NULL, 0, token);	
  	
	if (!coap_add_option(pdu, COAP_OPTION_OBSERVE,
                      COAP_OBSERVE_ESTABLISH, NULL)) {
	  coap_log(LOG_DEBUG, "failed to add option\n");
	}

	auto uri_path = name.toStdString();
	auto len = uri_path.length();

	coap_add_option(pdu, COAP_OPTION_URI_PATH, len,
                  (unsigned char *)uri_path.c_str());

  	coap_send(session, pdu);
}

void TrikCoapClient::coapRunOnce() {
	coap_run_once(ctx, 0);
}

static inline int check_token(coap_pdu_t *received, coap_binary_t token) {
	// !!! something wrong with token_length  
	return memcmp(received->token, token.s, token.length) == 0;
}

void TrikCoapClient::start(void) {
	std::cout << "started" << std::endl;
	coap_set_log_level(LOG_INFO);
	coap_startup();

	/* resolve destination address where server should be sent */
	if (resolve_address("192.168.1.103", "5683", &dst) < 0) {
		coap_log(LOG_CRIT, "failed to resolve address\n");
	}

	/* create CoAP context and a client session */
	ctx = coap_new_context(nullptr);

	if (!ctx || !(session = coap_new_client_session(ctx, nullptr, &dst,
			                                      COAP_PROTO_UDP))) {
		coap_log(LOG_EMERG, "cannot create client session\n");
	}

	/* coap_register_response_handler(ctx, response_handler); */
	coap_register_response_handler(ctx, [](coap_context_t *context, 
										   coap_session_t *session,
										   coap_pdu_t *sent,
									       coap_pdu_t *received, 
										   coap_tid_t id) {
		
		//coap_show_pdu(LOG_INFO, received);
		
		QVector<QString> sensors = { "A1", "A2", "A5", "A6", "D1", "D2" };

		QVector<coap_binary_t> tokens = {{2, (unsigned char*)"A1"},
										  {2, (unsigned char*)"A2"},
										  {2, (unsigned char*)"A5"},
										  {2, (unsigned char*)"A6"},
										  {2, (unsigned char*)"D1"},
										  {2, (unsigned char*)"D2"},
										};

		auto tok = (char*)received->token;
	
		for (int i = 0; i < tokens.size(); i++) {
			if (check_token(received, tokens[i])) {
				std::cout << i << std::endl;
				auto d = (char *)received->data;
				TrikCoapClient::instance->emitSignal(sensors[i], std::atoi(d));
			}
		}
	});

	timer = new QTimer();
	QObject::connect(timer, SIGNAL(timeout()), this, SLOT(coapRunOnce()));
	timer->setInterval(10);
	timer->start();
}

void TrikCoapClient::emitSignal(QString name, int data) {
	emit newData(name, data);
}

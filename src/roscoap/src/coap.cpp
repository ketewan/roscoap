#include <QApplication>
#include "coap.h"


TrikCoapClient::TrikCoapClient() {
};

TrikCoapClient::~TrikCoapClient() {
};

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

void TrikCoapClient::setPowerMotor(std::string name, int power) {
	/* construct CoAP message */
	auto pdu = coap_pdu_init(COAP_MESSAGE_CON,
		              COAP_REQUEST_GET,
		              0 /* message id */,
		              coap_session_max_pdu_size(session));
	if (!pdu) {
		coap_log(LOG_EMERG, "cannot create PDU\n");
	}

	coap_add_option(pdu, COAP_OPTION_URI_PATH, 6, (unsigned char *)"motors");
	
	auto query = name + "=" + std::to_string(power);
	unsigned char *buf = (unsigned char *)query.c_str();
	coap_add_option(pdu, COAP_OPTION_URI_QUERY, coap_opt_length(buf), coap_opt_value(buf));

	/* and send the PDU */
	coap_send(session, pdu);

	coap_run_once(ctx, 0);
}

void TrikCoapClient::start(void) {
	coap_set_log_level(LOG_DEBUG);
	coap_startup();

	/* resolve destination address where server should be sent */
	if (resolve_address("10.0.40.106", "5683", &dst) < 0) {
		coap_log(LOG_CRIT, "failed to resolve address\n");
	}

	/* create CoAP context and a client session */
	ctx = coap_new_context(nullptr);

	if (!ctx || !(session = coap_new_client_session(ctx, nullptr, &dst,
			                                      COAP_PROTO_UDP))) {
		coap_log(LOG_EMERG, "cannot create client session\n");
	}

	/* coap_register_response_handler(ctx, response_handler); */
	coap_register_response_handler(ctx, [](auto, auto, auto,
			                             coap_pdu_t *received,
			                             auto) {
			                            coap_show_pdu(LOG_INFO, received);
	});
}



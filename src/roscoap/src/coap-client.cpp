#include <QApplication>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <coap2/coap.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

coap_context_t  *ctx = nullptr;
coap_session_t *session = nullptr;
coap_address_t dst;
coap_pdu_t *pdu = nullptr;

int resolve_address(const char *host, const char *service, coap_address_t *dst) {
	struct addrinfo *res, *ainfo;
	struct addrinfo hints;
	int error, len=-1;

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

void send_msg() {
  /* construct CoAP message */
  pdu = coap_pdu_init(COAP_MESSAGE_CON,
                      COAP_REQUEST_GET,
                      0 /* message id */,
                      coap_session_max_pdu_size(session));
  if (!pdu) {
    coap_log( LOG_EMERG, "cannot create PDU\n" );
  }

  /* add a Uri-Path option */
  coap_add_option(pdu, COAP_OPTION_URI_PATH, 5,
                  reinterpret_cast<const uint8_t *>("hello"));

  /* and send the PDU */
  coap_send(session, pdu);

  coap_run_once(ctx, 0);
}

void start(void) {
	coap_set_log_level(LOG_DEBUG);

	coap_str_const_t ruri = { 5, (const uint8_t *)"hello" };

	coap_startup();

	/* resolve destination address where server should be sent */
	  if (resolve_address("localhost", "5683", &dst) < 0) {
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



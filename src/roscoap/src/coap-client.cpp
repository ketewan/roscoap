#include <QtGui/QApplication>
#include "trikCoapServer.h"

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
    goto finish;
  }

  /* add a Uri-Path option */
  coap_add_option(pdu, COAP_OPTION_URI_PATH, 5,
                  reinterpret_cast<const uint8_t *>("hello"));

  /* and send the PDU */
  coap_send(session, pdu);

  coap_run_once(ctx);
}

void start(void) {
		coap_set_log_level(LOG_DEBUG);
	
		coap_str_const_t ruri = { 5, (const uint8_t *)"hello" };

		coap_startup();

		/* resolve destination address where server should be sent */
		if (resolve_address("localhost", "5683", &dst) < 0) {
			coap_log(LOG_CRIT, "failed to resolve address\n");
			coap_free_context(this->ctx);
			coap_cleanup();
		}

		/* create CoAP context and a client session */
		ctx = coap_new_context(nullptr);

		if (!ctx || !(endpoint = coap_new_endpoint(ctx, &dst, COAP_PROTO_UDP))) {
			coap_log(LOG_EMERG, "cannot initialize context\n");
			coap_free_context(ctx);
			coap_cleanup();
		}

		resource = coap_resource_init(&ruri, 0);
		coap_register_handler(resource, COAP_REQUEST_GET, hnd_get_hello);
		coap_add_resource(ctx, resource);

		init_resources();
}

void led_callback(const std_msgs::Int32 cmd) {
	send_msg();
}

int main(int argc, char **argv) {
    // start Qt server
    int qargc = 2;
    const char *qargv[] = {"standalone_trik_node", "-qws"}; // todo: try QApplication::Tty for console app?
    QApplication app(qargc, (char **) qargv);

    // init ROS node
    ros::init(argc, argv, "standalone_trik_node");
    ros::NodeHandle nh;
    ros::Rate loopRate(1);

    ros::Subscriber sub = nh.subscribe("led_cmd", 10, led_callback);

    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}



#include <QApplication>
#include "trik.h"

TrikRosNode::TrikRosNode(ros::NodeHandle *nodeHandle) : nodeHandle_(nodeHandle) {
    auto rate = ros::Rate(nodeHandle->param("rate", 20));
    // sensorsTimer_ = nodeHandle->createTimer(ros::Duration(rate), &TrikRosNode::publishSensorsData, this);
    //ledSubscriber = nodeHandle->subscribe("cmd_led", 1, &TrikRosNode::ledCallback, this);
    velocitySubscriber = nodeHandle->subscribe("cmd_vel", 10, &TrikRosNode::velocityCallback, this);
	
	coap_thread = new QThread();
	coap_client = new TrikCoapClient();
	coap_client->moveToThread(coap_thread);

	coap_thread->start();

	QMetaObject::invokeMethod(coap_client, "start", Qt::QueuedConnection);
}

void TrikRosNode::velocityCallback(const geometry_msgs::Twist twist) {
	std::cout << "int vel clb" << std::endl;
    int leftPower = int((twist.linear.x - twist.angular.z) * 100);
    int rightPower = int((twist.linear.x + twist.angular.z) * 100);
	
    QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::DirectConnection, Q_ARG(QString, "M1"), Q_ARG(int, leftPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::DirectConnection, Q_ARG(QString, "M2"), Q_ARG(int, rightPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::DirectConnection, Q_ARG(QString, "M3"), Q_ARG(int, leftPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::DirectConnection, Q_ARG(QString, "M4"), Q_ARG(int, rightPower));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "trik");
    ros::NodeHandle nodeHandle;

    // start Qt server
    int qargc = 2;
    const char *qargv[] = {"standalone_trik_node", "-qws"}; 
    QApplication app(qargc, (char **) qargv);

    TrikRosNode trik(&nodeHandle);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return app.exec();

    return 0;
}



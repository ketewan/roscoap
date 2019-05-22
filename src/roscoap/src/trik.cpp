#include <QApplication>
#include "trik.h"

TrikRosNode::TrikRosNode(ros::NodeHandle *nodeHandle) : nodeHandle_(nodeHandle) {
    ledSubscriber = nodeHandle->subscribe("A1", 1, &TrikRosNode::ledCallback, this);
    velocitySubscriber = nodeHandle->subscribe("cmd_vel", 10, &TrikRosNode::velocityCallback, this);
	
	coap_thread.reset(new QThread());
	coap_client = new TrikCoapClient();
	coap_client->moveToThread(coap_thread.data());

	coap_thread->start();

	QObject::connect(coap_client, &TrikCoapClient::newData, this, &TrikRosNode::publishSensorData);

	QMetaObject::invokeMethod(coap_client, "start", Qt::QueuedConnection);

	sensorPorts = {"A1", "A2", "A5", "A6", "D1", "D2"};

	for (auto port : sensorPorts) {
		publishers[port] = nodeHandle_->advertise<std_msgs::Int32>(port.toStdString(), 10);
	}

    subscribeForSensors();
}

void TrikRosNode::ledCallback(const std_msgs::Int32 dist) {
	if (dist.data < 20)
		QMetaObject::invokeMethod(coap_client, "changeLed", Qt::QueuedConnection,
		                           Q_ARG(QString, "green"));
	else
		QMetaObject::invokeMethod(coap_client, "changeLed", Qt::QueuedConnection,
		                           Q_ARG(QString, "red"));
}

TrikRosNode::~TrikRosNode() {
	// coap_thread->requestInterruption();
}

void TrikRosNode::subscribeForSensors() {
	//for (auto port : sensorPorts) {
	//	QMetaObject::invokeMethod(coap_client, "subscribeForSensor", Qt::QueuedConnection,
	//	                           Q_ARG(QString, port));
	//}

	QMetaObject::invokeMethod(coap_client, "subscribeForSensor", Qt::QueuedConnection,
		                           Q_ARG(QString, "A2"));
QMetaObject::invokeMethod(coap_client, "subscribeForSensor", Qt::QueuedConnection,
		                           Q_ARG(QString, "A3"));
}

void TrikRosNode::publishSensorData(QString name, int data) {
	std_msgs::Int32 msg;
	msg.data = data;
	//std::cout << "Going to publish" << std::endl;
	publishers[name].publish(msg);
}

void TrikRosNode::velocityCallback(const geometry_msgs::Twist twist) {
    int leftPower = int((twist.linear.x - twist.angular.z) * 100);
    int rightPower = int((twist.linear.x + twist.angular.z) * 100);

    QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::QueuedConnection, Q_ARG(QString, "M1"), Q_ARG(int, leftPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::QueuedConnection, Q_ARG(QString, "M2"), Q_ARG(int, rightPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::QueuedConnection, Q_ARG(QString, "M3"), Q_ARG(int, leftPower));
	QMetaObject::invokeMethod(coap_client, "setPowerMotor", Qt::QueuedConnection, Q_ARG(QString, "M4"), Q_ARG(int, rightPower));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "trik");
    ros::NodeHandle nodeHandle;

    // start Qt server
    int qargc = 2;
    const char *qargv[] = {"standalone_trik_node", "-qws"}; 
    QApplication app(qargc, (char **) qargv);

    TrikRosNode trik(&nodeHandle);

	QTimer* timer = new QTimer();
    
	QObject::connect(timer, &QTimer::timeout, [timer]() {
			if (ros::ok())
				ros::spinOnce();
			else 
				timer->stop();
	});

    if (ros::ok())
		timer->start(0);

    return app.exec();
}



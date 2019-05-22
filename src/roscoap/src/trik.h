#pragma once

#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include "coap.h"
#include <QMap>

class TrikRosNode : public QObject {
	Q_OBJECT
	
    public:
		~TrikRosNode();

        TrikRosNode(ros::NodeHandle *nodeHandle);

        void subscribeForSensors();

        void ledCallback(const std_msgs::Int32 cmd);

        void velocityCallback(const geometry_msgs::Twist twist);

	signals:
		void changePower(QString name, int power);
	
	public slots:
		void publishSensorData(QString sensor, int data);

	private:
        ros::NodeHandle *nodeHandle_;
        ros::Subscriber ledSubscriber;
        ros::Subscriber velocitySubscriber;
		QScopedPointer<QThread> coap_thread;
		TrikCoapClient *coap_client;
		QVector<QString> sensorPorts;
		QMap<QString, ros::Publisher> publishers;
};


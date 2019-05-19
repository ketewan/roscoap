#pragma once

#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include "coap.h"

class TrikRosNode : public QObject {
	Q_OBJECT

    public:
		virtual ~TrikRosNode() {};
        TrikRosNode(ros::NodeHandle *nodeHandle);

        void publishSensorsData(const ros::TimerEvent &event);

        void ledCallback(const std_msgs::Int32 cmd);

        void velocityCallback(const geometry_msgs::Twist twist);

	signals:
		void changePower(QString name, int power);
	
	private:
        ros::NodeHandle *nodeHandle_;
        ros::Timer timer;
        ros::Subscriber ledSubscriber;
        ros::Subscriber velocitySubscriber;
		QThread *coap_thread;
		TrikCoapClient *coap_client;
};


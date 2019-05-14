#pragma once

#include <QThread>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include "coap.h"

class TrikRosNode {
    public:
        TrikRosNode(ros::NodeHandle *nodeHandle);

        void publishSensorsData(const ros::TimerEvent &event);

        void ledCallback(const std_msgs::Int32 cmd);

        void velocityCallback(const geometry_msgs::Twist twist);

	private:
        ros::NodeHandle *nodeHandle_;
        ros::Timer timer;
        ros::Subscriber ledSubscriber;
        ros::Subscriber velocitySubscriber;
		QThread *coap_thread;
		TrikCoapClient *coap_client;
};


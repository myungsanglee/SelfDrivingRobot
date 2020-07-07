/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

#include "../include/gui_subscriber/main_window.hpp"
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gui_subscriber/qnode.hpp"
#include <ros/ros.h>
#include <ros/network.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>

namespace enc = sensor_msgs::image_encodings;

namespace gui_subscriber {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_subscriber");
	if ( ! ros::master::check() ) {
		return false;
	}
        //ros::start(); // explicitly needed since our nodehandle is going out of scope.
        //ros::NodeHandle n;
	// Add your ros communications here.
  /*chatter_subscriber = n.subscribe("chatter",1000,&QNode::myCallback,this);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe("image_pub",1,&QNode::myCallback_img,this);//图片尝试
        start();
        */
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    //ros::Rate loop_rate(1);
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
	ros::init(remappings,"gui_subscriber");
        if ( ! ros::master::check() ) {
		return false;
	}
        ros::NodeHandle n;
        ros::NodeHandle nn;
        //chatter_subscriber = n.subscribe("chatter",1000,&QNode::myCallback,this);
        image_transport::ImageTransport it(n);
        image_transport::Subscriber image_sub;
        image_sub = it.subscribe("nano/camera/color_image",1,&QNode::myCallback_img,this);

        image_transport::ImageTransport itt(nn);
        image_transport::Subscriber image_sub_pi;
        image_sub_pi = itt.subscribe("pi/image",1,&QNode::myCallback_img_pi,this);

        chatter_subscriber = n.subscribe("nano/event",1000,&QNode::myCallback,this);
        ros::start();
        start();
        ros::spin();
        //loop_rate.sleep();
        std::cout<<"ROS shutdown,proceding to clode the gui."  <<std::endl;
        Q_EMIT rosShutdown();
        //std::cout<<"ROS shutdown,proceding to clode the gui."  <<std::endl;
        //Q_EMIT rosShutdown();
        //ros::start(); // explicitly needed since our nodehandle is going out of scope.
        //ros::NodeHandle n;
	// Add your ros communications here.
  /*chatter_subscriber = n.subscribe("chatter",1000,&QNode::myCallback,this);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe("image_pub",1,&QNode::myCallback_img,this);//subscribe image
    */
//start();
	return true;
}
void QNode::log( const LogLevel &level, const std_msgs::String &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " <<"recevied valude is:"<< msg.data;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
        logging_model_msg << msg.data;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " <<"recevied valude is:"<< msg.data;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " <<"recevied valude is:"<< msg.data;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " <<"recevied valude is:"<< msg.data;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::myCallback_img(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
      printf("nano\n");
      Q_EMIT imageSignal(img);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::myCallback_img_pi(const sensor_msgs::ImageConstPtr& msg_pi)
{
    try
    {
      //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      img = cv_bridge::toCvShare(msg_pi, "mono8")->image;
      printf("pi\n");
      Q_EMIT imageSignal_pi(img);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_pi->encoding.c_str());
    }
}


void QNode::myCallback(const std_msgs::String &message_holder)
{
  log(Info,message_holder);
}



}  // namespace gui_subscriber

#ifndef AUTORACE_H
#define AUTORACE_H

#include <iostream>
#define ON 1

class ControlLane
{
	public:
		//생성자
		ControlLane();
		
		int lastError = 0;
		float angular_z = 0;
		int count = 0;
		int wait_flag = 0;
		int right_laser = ON;
		double distance = 0;
		int parking1 = 0;
		int parking2 = 0;
		int stopLinestop = 0;
		
		void eventCallback(const std_msgs::String::ConstPtr& nano_event);
		void centerCallback(const std_msgs::Int32::ConstPtr& sub_center);
		void stopCallback(const std_msgs::Int32::ConstPtr& sub_stop);
		void stateAndEvent();
		void publishCmdVel(ros::Publisher *cmd_vel);
		void firstParking(ros::Publisher *cmd_vel, int, int, int);
		void secondParking(ros::Publisher *cmd_vel, int, int, int);

	private:
		int error = 0;
		float Kp = 0.0025;
		float Kd = 0.02;
		float angle = 0.5;
};

#endif

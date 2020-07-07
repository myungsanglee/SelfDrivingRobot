#include <ros/ros.h>
#include <vector>
#include <wiringPi.h>
#include <unordered_map>
#include <unistd.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "autorace.h"
#include "tof.h"
#include "pthread.h"

using namespace std;

#define SPEED_0 0.0
#define SPEED_N 0.05
#define SPEED_P 0.02
#define SPEED_L -0.01
#define ANGLE_0 0.0

#define ON 1
#define OFF 0

#define DBG

string event = "clear";
bool RealStop = false;
bool centerThread = false;
bool parking_finish = false;
bool parking_thread = true;

float MAX_VEL = 0;
int stop = 0;
int center = 0;
int pub_count = 0;

void* stopLineStopThread(void *ret);
void* centerIsCenter(void *ret);
void* parkingThread(void *ret);

enum Event
{
    person,
    red,
    green,
    no_right_turn,
    speed_up,
    school_zone,
    school_zone_off,
    roundabout,
    tunnel,
    first_parking,
    second_parking,
    turtlebot,
    clear
};

enum State
{
    driving_start,
    Drivex0_red,
    Drivex0_person,
    Drivex1,
    Drivex2,
    Drivehalf,
    Roundabout,
    FParking,
    SParking,
    NoRightTurn
};

unordered_map<string, int> event_map = {
    {"person", person},
    {"red", red},
    {"green", green},
    {"no_right_turn", no_right_turn},
    {"speed_up", speed_up},
    {"school_zone", school_zone},
    {"school_zone_off", school_zone_off},
    {"roundabout", roundabout},
    {"tunnel", tunnel},
    {"first_parking", first_parking},
    {"second_parking", second_parking},
    {"clear", clear},
    {"driving_start", driving_start},
    {"turtlebot", turtlebot}
};

enum POSISION_ARRANGE {
	find_pos = 0,
	rearrange
};

enum PARKING {
	pause_0 = 0,
	turn_right,
	turn_right2,
	turn_right3,
	pause_1,
	turn_left1,
	turn_left2,
	pause_2,
	nothing
};

enum WAYOUT {
	start = 0,
	out_right1,
	out_right2,
	out_right3,
	out_left1,
	out_left2,
	out_pause,
	go_back,
	go_left,
	go_out_left,
	go_out_right,
	lane_follow
};

enum POSISION_ARRANGE POS_ARRANGE = find_pos;
enum PARKING PARKING_ANGLE = pause_0;
enum WAYOUT OUT_ANGLE = start;

enum State state = driving_start;
enum State pre_state = driving_start;
enum Event event_enum;

//FUNCTION

ControlLane::ControlLane() : lastError(0)
{
}

void ControlLane::centerCallback(const std_msgs::Int32::ConstPtr& sub_center)
{
    int err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;
    
    center = sub_center->data;
    
    if (centerThread)
    {
	if (err = pthread_create(&pth, NULL, centerIsCenter, (void*)&input) < 0)
	{
	    perror("Thread1 error : ");
	    exit(2);
	}
	pthread_detach(pth);
    }
}

void ControlLane::eventCallback(const std_msgs::String::ConstPtr& nano_event)
{
    event = nano_event->data.c_str();
    if (state == Roundabout)
    {
	if (event_map[event] == turtlebot)
	{
	    MAX_VEL = 0.05;
	    stop = 0;
	    centerThread = true;
	    state = Drivex1;
	}
    }
}

void ControlLane::stopCallback(const std_msgs::Int32::ConstPtr& sub_stop)
{
    int err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;
    
    enum State state;
    enum Event event_enum;

    stopLinestop = sub_stop->data;
    
    if (stopLinestop && RealStop)
    {
	if (err = pthread_create(&pth, NULL, stopLineStopThread, (void*)&input) < 0)
	{
	    perror("Thread1 error : ");
	    exit(2);
	}
	pthread_detach(pth);
    }
}

void ControlLane::stateAndEvent()
{ 
    switch(state)
    {
	/****driving_start****/
	case driving_start:
#ifdef DBG
	    cout << "STATE = driving_start" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == green)
		state = Drivex1;
	    else if (event_map[event] == clear)
		state = driving_start;
	    break;
	
	/****Drivex1****/
	case Drivex1:
#ifdef DBG
	    cout << "STATE = Drivex1" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.05;
	    stop = 0;
	    if (event_map[event] == person)
	    {
		pre_state = Drivex1;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == red)
		state = Drivex0_red;
	    else if (event_map[event] == no_right_turn)
		state = NoRightTurn;
	    else if (event_map[event] == first_parking)
		state = FParking;
	    else if (event_map[event] == second_parking)
		state = SParking;
	    else if (event_map[event] == roundabout)
		state = Roundabout;
	    else
		state = Drivex1;
	    break;
	
	/****Drivex2****/
	case Drivex2:
#ifdef DBG
	    cout << "STATE = Drivex2" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.1;
	    stop = 0;
	    if (event_map[event] == school_zone)
		state = Drivehalf;
	    else if (event_map[event] == person)
	    {
		pre_state = Drivex2;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == clear)
		state = Drivex2;
	    break;
	
	/****Drivehalf****/
	case Drivehalf:
#ifdef DBG
	    cout << "STATE = Drivehalf" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.02;
	    stop = 0;
	    if (event_map[event] == school_zone_off)
		state = Drivex1;
	    else if (event_map[event] == person)
	    {
		pre_state = Drivehalf;
		state = Drivex0_person;
	    }
	    else if (event_map[event] == clear)
		state = Drivehalf;
	    break;
	
	/****Drivex0_person****/
	case Drivex0_person:
#ifdef DBG
	    cout << "STATE = Drivex0_person" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == clear)
		state = pre_state;
	    else if (event_map[event] == green)
	    {
		state = Drivex1;
		parking1 = 0;
		parking2 = 0;
	    }
	    break;
	
	/****Drivex0_red****/
	case Drivex0_red:
#ifdef DBG
	    cout << "STATE = Drivex0_red" << endl;
#endif
	    RealStop = false;
	    MAX_VEL = 0.0;
	    stop = 1;
	    if (event_map[event] == clear)
		state = Drivex0_red;
	    else if (event_map[event] == green)
	    {
		MAX_VEL = 0.05;
		stop = 0;
		state = Drivex1;
	    }
	    break;
	
	case Roundabout:
	    cout << "STATE = Roundabout" << endl;
	    RealStop = true;
	    if (event_map[event] == clear)
		state = Roundabout;
	    if (event_map[event] == turtlebot)
	    {
		MAX_VEL = 0.05;
		stop = 0;
		centerThread = true;
		state = Drivex1;
	    }
	    else if (event_map[event] == person)
	    {
		pre_state = NoRightTurn;
		state = Drivex0_person;
	    }
	    break;
	    
	/****FParking****/
	case FParking:
	    RealStop = false;
	    if (event_map[event] == clear)
		state = FParking;
	    else if (event_map[event] == second_parking)
		state = SParking;
	    else if (event_map[event] == person)
	    {
		pre_state = NoRightTurn;
		state = Drivex0_person;
	    }
	    break;
	    
	/****SParking****/
	case SParking:
	    RealStop = false;
	    if (event_map[event] == clear)
		state = SParking;
	    else if (event_map[event] == red)
		state = Drivex0_red;
	    else if (event_map[event] == person)
	    {
		pre_state = NoRightTurn;
		state = Drivex0_person;
	    }
	    break;
	
	case NoRightTurn:
	    RealStop = true;
	    MAX_VEL = 0.05;
	    stop = 0;
	    if (event_map[event] == clear)
		state = NoRightTurn;
	    else if (event_map[event] == speed_up)
		state = Drivex2;
	    else if (event_map[event] == person)
	    {
		pre_state = NoRightTurn;
		state = Drivex0_person;
	    }
	    break;
    }

#ifdef DBG
    cout << "EVENT = " << event << endl;
    cout << "MAX_VEL = " << MAX_VEL << endl;
#endif
}

void ControlLane::publishCmdVel(ros::Publisher *cmd_vel)
{
    geometry_msgs::Twist twist;
    
    error = center - 327;

    if(stop)
	    angular_z = 0;
    else
	    angular_z = Kp * error + Kd * (error - lastError);
    
    lastError = error;
    twist.linear.x = std::min(MAX_VEL * pow(1 - abs(error) / 327, 2.2), 0.2);
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
    
    cmd_vel -> publish(twist);
}

void ControlLane::firstParking(ros::Publisher *cmd_vel, int i2c_bus, int i2c_address, int long_range)
{   
    enum State state;
    enum Event event_enum;  
    geometry_msgs::Twist twist;

    distance = tofReadDistance();
    ROS_INFO("distance = %f", distance);
    
    int err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;

    error = center - 327;
    angular_z = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    if (parking1 == 0)
    {
	digitalWrite(20, ON);  //right
	digitalWrite(21, OFF);   //back
	right_laser = ON;
	tofInit(i2c_bus, i2c_address, long_range);
	count = 0;
	wait_flag = 0;
	POS_ARRANGE = find_pos;
	parking1 ++;
	pub_count = 0;
	sleep(1);
    }

    /*****POS_ARRANGE*****/
    //right_laser = ON, back_laser = OFF
    if(right_laser == ON)
    {
        //Find Position
        if(POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_N;
            twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
            cmd_vel -> publish(twist);
            if(distance > 140.0 & distance < 450)
                count += 1;
#ifdef DBG
	    cout << "POS_ARRANGE = 0" << endl;
#endif
        }
        
        ROS_INFO("count = %d\n", count);
        
        //NO Obstacle
        if (distance < 130.0 && count > 25 && POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
            POS_ARRANGE = rearrange;
#ifdef DBG
	    cout << "POS_ARRANGE = 1" << endl;
#endif
        }
        
        //Rearrange
        if (POS_ARRANGE == rearrange)
        {
            twist.linear.x = SPEED_L;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
#ifdef DBG
	    cout << "POS_ARRANGE = 2" << endl;
#endif
            if(distance > 110 && distance < 450 && count > 25)
            {
#ifdef DBG
		cout << "BACK LASER ON!" << endl;
#endif
                digitalWrite(20, OFF);  //right
                digitalWrite(21, ON);   //back
                right_laser = OFF;
                tofInit(i2c_bus, i2c_address, long_range);
                PARKING_ANGLE = pause_0;
		OUT_ANGLE = start;
            }
        }
    }
    /*****PARKING*****/
    //right_laser = OFF, back_laser = ON
    else 
    {
        //sleep once
        if(wait_flag == 0)
        {
            sleep(1);
            wait_flag = 1;
        }
        
        //PARKING START
        if (PARKING_ANGLE == pause_0)
        {
#ifdef DBG
            cout <<  "waiting for back laser" << endl;
#endif
            if (distance > 300)
                PARKING_ANGLE = turn_right;
        }
	else if (PARKING_ANGLE == turn_right)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 0" << endl;
#endif
	    twist.linear.x = -SPEED_P;
	    twist.angular.z = 0.1;
	    cmd_vel -> publish(twist);
	    if (distance < 110)
		PARKING_ANGLE = pause_1;
	}
	else if (PARKING_ANGLE == pause_1)
	{
	    sleep(1);
            PARKING_ANGLE = turn_left1;
        }
	else if (PARKING_ANGLE == turn_left1)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 1" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = -0.25;
            cmd_vel -> publish(twist);
            if(distance > 130)
                PARKING_ANGLE = turn_left2;
        }
	else if (PARKING_ANGLE == turn_left2)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 2 " << endl;
#endif
	    twist.linear.x = -SPEED_P;
	    twist.angular.z = -0.25;
	    cmd_vel -> publish(twist);
	    if (distance < 60)
	    {
		PARKING_ANGLE = pause_2;
	    }
	}
	else if(PARKING_ANGLE == pause_2)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 3 " << endl;
#endif
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
	    sleep(3);
	    OUT_ANGLE = out_right1;
	    PARKING_ANGLE = nothing;
        }
	
	if(OUT_ANGLE == out_right1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 0" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.3;
	    cmd_vel -> publish(twist);
	    if (distance > 90)
	    {
		OUT_ANGLE = out_right2;
	    }
	}
	else if(OUT_ANGLE == out_right2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.3;
	    cmd_vel -> publish(twist);
	    if (distance < 80)
	    {
		OUT_ANGLE = out_left1;
	    }
	}
	else if(OUT_ANGLE == out_left1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = 0.05;
	    cmd_vel -> publish(twist);
	    if (distance > 110)
	    {
		OUT_ANGLE = out_pause;
	    }
	}
	else if (OUT_ANGLE == out_pause)
	{
	    sleep(1);
            OUT_ANGLE = out_left2;
	}
	else if (OUT_ANGLE == out_left2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 2" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = -0.1;
	    cmd_vel -> publish(twist);
	    if (distance > 8000)
	    {
		if (parking_thread)
		{
		    if (err = pthread_create(&pth, NULL, parkingThread, (void*)&input) < 0)
		    {
			perror("Thread1 error : ");
			exit(2);
		    }
		    pthread_detach(pth);
		}
	    }
	}
	else if (OUT_ANGLE == lane_follow)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 3" << endl;
#endif
	    twist.linear.x = SPEED_N;
	    twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
	    cmd_vel -> publish(twist);
	    parking_finish = true;
	    if (pub_count != 0)
		parking_finish = false;
	    pub_count++;
	}
    }
    cmd_vel -> publish(twist);
}

void ControlLane::secondParking(ros::Publisher *cmd_vel, int i2c_bus, int i2c_address, int long_range)
{
    enum State state;
    enum Event event_enum;
    geometry_msgs::Twist twist;

    distance = tofReadDistance();
    ROS_INFO("distance = %f", distance);

    error = center - 327;
    angular_z = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    if (parking2 == 0)
    {
	parking_finish = false;
	digitalWrite(20, ON);  //right
	digitalWrite(21, OFF);   //back
	right_laser = ON;
	tofInit(i2c_bus, i2c_address, long_range);
	count = 0;
	wait_flag = 0;
	POS_ARRANGE = find_pos;
	OUT_ANGLE = start;
	parking2 ++;
	pub_count = 0;
	sleep(1);
    }

    /*****POS_ARRANGE*****/
    //right_laser = ON, back_laser = OFF
    if(right_laser == ON)
    {
        //Find Position
        if(POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_N;
            twist.angular.z = angular_z < 0 ? -max(angular_z, -angle) : -min(angular_z, angle);
            cmd_vel -> publish(twist);
            if(distance > 140.0 & distance < 450)
                count += 1;
#ifdef DBG
	    cout << "POS_ARRANGE = 0" << endl;
#endif
        }
        
        ROS_INFO("count = %d\n", count);
        
        //NO Obstacle
        if (distance < 130.0 && count > 15 && POS_ARRANGE == find_pos)
        {
            twist.linear.x = SPEED_0;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
            POS_ARRANGE = rearrange;
#ifdef DBG
	    cout << "POS_ARRANGE = 1" << endl;
#endif
        }
        
        //Rearrange
        if (POS_ARRANGE == rearrange)
        {
            twist.linear.x = SPEED_L;
            twist.angular.z = ANGLE_0;
            cmd_vel -> publish(twist);
#ifdef DBG
	    cout << "POS_ARRANGE = 2" << endl;
#endif
            if(distance > 110 && distance < 450 && count > 15)
            {
#ifdef DBG
		cout << "BACK LASER ON!" << endl;
#endif
                digitalWrite(20, OFF);  //right
                digitalWrite(21, ON);   //back
                right_laser = OFF;
                tofInit(i2c_bus, i2c_address, long_range);
                PARKING_ANGLE = pause_0;
            }
        }
    }
    /*****PARKING*****/
    //right_laser = OFF, back_laser = ON
    else 
    {
        //sleep once
        if(wait_flag == 0)
        {
            sleep(1);
            wait_flag = 1;
        }
        
        //PARKING START
        if (PARKING_ANGLE == pause_0)
        {
#ifdef DBG
            cout <<  "waiting for back laser" << endl;
#endif
            if (distance > 300)
	    {
                PARKING_ANGLE = turn_right;
		OUT_ANGLE = start;
	    }
        }
        else if (PARKING_ANGLE == turn_right)
        {
#ifdef DBG
	    cout << "PARKING_ANGLE = 0" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = 0.1;
            cmd_vel -> publish(twist);
            if (distance < 90)
                PARKING_ANGLE = pause_1;
        }
	else if(PARKING_ANGLE == turn_right2)
	{
#ifdef DBG
	    cout << "PARKING_ANGLE = 1" << endl;
#endif
            twist.linear.x = -SPEED_P;
            twist.angular.z = 0.1;
            cmd_vel -> publish(twist);
            if (distance > 110)
                PARKING_ANGLE = pause_1;
	}
	else if(PARKING_ANGLE == pause_1)
        {
            sleep(3);
            OUT_ANGLE = out_right1;
	    PARKING_ANGLE = nothing;
        }
	
	if(OUT_ANGLE == out_right1)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 0" << endl;
#endif
	    twist.linear.x = SPEED_N;
	    twist.angular.z = ANGLE_0;
	    cmd_vel -> publish(twist);
	    if(distance > 160)
		OUT_ANGLE = out_right2;
	}
	else if(OUT_ANGLE == out_right2)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 1" << endl;
#endif
	    twist.linear.x = SPEED_P;
	    twist.angular.z = -0.15;
	    cmd_vel -> publish(twist);
	    parking_finish = true;
	    if (pub_count != 0)
		parking_finish = false;
	    pub_count++;
	    if(distance > 8000)
	    {
		OUT_ANGLE = lane_follow;
		RealStop = true;
	    }
	}
	/*else if(OUT_ANGLE == out_pause)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 2" << endl;
#endif
	    sleep(3);
	}*/
	else if (OUT_ANGLE == lane_follow)
	{
#ifdef DBG
	    cout << "OUT_ANGLE = 2" << endl;
#endif
	    RealStop = true;
	    twist.linear.x = SPEED_N;
	    twist.angular.z = angular_z < 0 ? -std::max(angular_z, -angle) : -std::min(angular_z, angle);
	    cmd_vel -> publish(twist);
	}
    }
    
    cmd_vel -> publish(twist);
}

void* stopLineStopThread(void *input1)
{
    int temp;
    
    if (state == Roundabout)
    {
	if(event_map[event] == clear)
	{
	    MAX_VEL = 0.0;
	    stop = 1;
	    parking_finish = true;
	    if (pub_count != 0)
		parking_finish = false;
	    pub_count++;
	}
	if (event_map[event] == turtlebot)
	{
	    MAX_VEL = 0.05;
	    stop = 0;
	    centerThread = true;
	    state = Drivex1;
	}
    }
    if (state == Drivex0_red)
    {
	if (event_map[event] == clear)
	{
	    MAX_VEL = 0.0;
	    stop = 1;
	}
    }
    
    else
    {
	sleep(5);
	if (state == NoRightTurn)
	    centerThread = true;
    }
    
    return (void*)temp;
}

void* centerIsCenter(void *input2)
{
    int temp;
    center = 327;
    cout << "Center is 327! \n" << endl; 
    usleep(4500000);
    centerThread = false;
    
    return (void*)temp;
}

void* parkingThread(void *input3)
{
    int temp;
    printf("PARKING THREAD!!!!!!!!!!!!!!!!!!!");
    sleep(4);
    parking_thread = false;
    OUT_ANGLE = lane_follow;
    
    return (void*)temp;
}

//MAIN

int main(int argc, char** argv)
{
    int i2c_bus = 0;
    int i2c_address = 0;
    int long_range = 0;
    double poll_rate = 0;

    ros::init(argc, argv, "control_lane");
    ros::NodeHandle nh, nh_priv("~");
    
    ControlLane *controllane = new ControlLane();
    
    ros::Subscriber sub_lane = nh.subscribe<std_msgs::Int32>("pi/center", 1, &ControlLane::centerCallback, controllane);
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("nano/event", 1, &ControlLane::eventCallback, controllane); 
    ros::Subscriber sub_stop = nh.subscribe<std_msgs::Int32>("pi/stop", 1, &ControlLane::stopCallback, controllane);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher pub_parking_fin = nh.advertise<std_msgs::String>("pi/event", 1);

    ros::Rate loop_rate(10);

    nh_priv.param("long_range", long_range, 0);
    nh_priv.param("poll_rate", poll_rate, 100.0);
    nh_priv.param("i2c_bus", i2c_bus, 1);
    nh_priv.param("i2c_address", i2c_address, 0x29);
    
    wiringPiSetupGpio();			
    pinMode(20, OUTPUT);		
    pinMode(21, OUTPUT);
    digitalWrite(20, ON);
    digitalWrite(21, OFF);
    
    tofInit(i2c_bus, i2c_address, long_range);
    
    while(nh.ok())
    {
	controllane -> stateAndEvent();
	
	if (RealStop)
	    printf("RealStop = 1\n");
	else
	    printf("RealStop = 0\n");
	
	 switch(state)
	{
	    case FParking:
		cout << "First Parking" << endl;
		controllane -> firstParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
		break;
	    case SParking:
		cout << "Seconde Parking" << endl;
		controllane -> secondParking(&pub_cmd_vel, i2c_bus, i2c_address, long_range);
		break;
	    default:
		controllane -> publishCmdVel(&pub_cmd_vel);
		break;
	}
	
	if(parking_finish)
	{
	    std_msgs::String ps;
	    stringstream ss;
	    ss << "parking_finish";
	    ps.data = ss.str();
	    pub_parking_fin.publish(ps);
	}
	    
	ros::spinOnce();
	loop_rate.sleep();
    }
    
    return 0;
}

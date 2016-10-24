#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#define VEHICLE_NUM 2
#define JOY_NUM 2
int g_vehicle_num;
int g_joy_num;
class Commander
{
private:

	std::vector<ros::Publisher> m_pubAttsp_v, m_pubPossp_v, m_pubVelff_v, m_pubAccff_v;
	std::vector<ros::Subscriber> joy_sub_v;
	enum_mode m_flight_mode;
	enum_state m_flight_state;
	PID m_pidX, m_pidY, m_pidZ, m_pidYaw;
	float m_velff_xy_P, m_velff_z_P;
	struct M_Joy
	{
		bool curr_buttons[14];
		bool changed_buttons[14];
		float axes[4];
	};
	std::vector<M_Joy> m_joy_v;
	struct M_Ctrl
	{
		geometry_msgs::Vector3Stamped pos_sp;
	//	geometry_msgs::Vector3Stamped vel_sp;
		geometry_msgs::Vector3Stamped vel_ff;
		geometry_msgs::Vector3Stamped acc_ff;
		geometry_msgs::Vector3Stamped att_sp;//r,p,y
		float throttle;
	};
	std::vector<M_Ctrl> m_ctrl_v;
	struct M_Est
	{
		geometry_msgs::Vector3Stamped pos_est;
		geometry_msgs::Vector3Stamped vel_est;
		float yaw_est;
	};
	std::vector<M_Est> m_est_v;
public:
	Commander(const ros::NodeHandle& nh)
	:m_pubAttsp_v(g_vehicle_num)
	,m_pubPossp_v(g_vehicle_num)
	,m_pubVelff_v(g_vehicle_num)
	,m_pubAccff_v(g_vehicle_num)
	,joy_sub_v(g_joy_num)
	,m_joy_v(g_joy_num)
	,m_ctrl_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_pidX(
		get(n, "PIDs/X/kp"),
		get(n, "PIDs/X/kd"),
		get(n, "PIDs/X/ki"),
		get(n, "PIDs/X/kpp"),
		get(n, "PIDs/X/minOutput"),
		get(n, "PIDs/X/maxOutput"),
		get(n, "PIDs/X/integratorMin"),
		get(n, "PIDs/X/integratorMax"),
		"x")
	,m_pidY(
		get(n, "PIDs/Y/kp"),
		get(n, "PIDs/Y/kd"),
		get(n, "PIDs/Y/ki"),
		get(n, "PIDs/Y/kpp"),
		get(n, "PIDs/Y/minOutput"),
		get(n, "PIDs/Y/maxOutput"),
		get(n, "PIDs/Y/integratorMin"),
		get(n, "PIDs/Y/integratorMax"),
		"y")
	,m_pidZ(
		get(n, "PIDs/Z/kp"),
		get(n, "PIDs/Z/kd"),
		get(n, "PIDs/Z/ki"),
		get(n, "PIDs/Z/kpp"),
		get(n, "PIDs/Z/minOutput"),
		get(n, "PIDs/Z/maxOutput"),
		get(n, "PIDs/Z/integratorMin"),
		get(n, "PIDs/Z/integratorMax"),
		"z")
	,m_pidYaw(
		get(n, "PIDs/Yaw/kp"),
		get(n, "PIDs/Yaw/kd"),
		get(n, "PIDs/Yaw/ki"),
		get(n, "PIDs/Yaw/kpp"),
		get(n, "PIDs/Yaw/minOutput"),
		get(n, "PIDs/Yaw/maxOutput"),
		get(n, "PIDs/Yaw/integratorMin"),
		get(n, "PIDs/Yaw/integratorMax"),
		"yaw")
	{
		
		m_flight_mode = AttCtrl;
		m_flight_state = Idle;
		m_velff_xy_P = 0.6;
		m_velff_z_P = 0.5;
		char msg_name[50];

		for(int i=0;i<g_vehicle_num;i++){

			sprintf(msg_name,"/vehicle%d/manuel_att_sp",i);
			m_pubAttsp_v[i] = nh.advertise<geometry_msgs::Vector3Stamped>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/pos_sp",i);
			m_pubPossp_v[i] = nh.advertise<geometry_msgs::Vector3Stamped>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/vel_ff",i);
			m_pubVelff_v[i] = nh.advertise<geometry_msgs::Vector3Stamped>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/acc_ff",i);
			m_pubAccff_v[i] = nh.advertise<geometry_msgs::Vector3Stamped>(msg_name, 1);
			
		}
		joy_sub[0] = nh.subscribe<sensor_msgs::Joy>("/joygroup0/joy",5,&Commander::joyCallback0,this);
		if(g_joy_num > 1){
			joy_sub[1] = nh.subscribe<sensor_msgs::Joy>("/joygroup1/joy",5,&Commander::joyCallback1,this);
		}
	}
	void run(double frequency)
	{
		ros::NodeHandle node;
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Commander::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvents& e)
	{
		float dt = e.current_real.toSec() - e.last_real.toSec();
		if(m_joy[0].changed_buttons[0] == true && m_joy[0].curr_buttons[0] == true){
			m_joy[0].changed_buttons[0] = false;
			m_flight_mode = PosCtrl;
		}
		else if(m_joy[0].changed_buttons[1] == true && m_joy[0].curr_buttons[1] == true){
			m_joy[0].changed_buttons[1] = false;
			m_flight_mode = AttCtrl;
		}
		else if(m_joy[0].changed_buttons[2] == true && m_joy[0].curr_buttons[2] == true){
			m_joy[0].changed_buttons[2] = false;
			m_flight_mode = TrjCtrl;
		}
		for(int i=0;i<JOY_NUM;i++){
			switch(m_flight_mode){
			case AttCtrl:{
			//	static float yaw_sp;
				m_ctrl[i].att_sp.x = -m_joy[i].axes[0] * 30 * DEG2RAD;//+-1
				m_ctrl[i].att_sp.y = m_joy[i].axes[1] * 30 * DEG2RAD;
				float yaw_move_rate = m_joy[i].axes[2] * 20 * DEG2RAD;
				m_ctrl[i].att_sp.z += yaw_move_rate;
				m_ctrl[i].throttle = m_joy[i].axes[3];//0-1
				if(m_ctrl[i].throttle<0)
					m_ctrl[i].throttle=0;
				m_pubAttsp[i].publish(m_ctrl[i].att_sp);
			}
			break;
			case PosCtrl:{
				float pos_move_rate[3];
				pos_move_rate[0] = m_joy[i].axes[0] * 2.0;
				pos_move_rate[1] = m_joy[i].axes[1] * 2.0;
				pos_move_rate[2] = m_joy[i].axes[3] * 1.0;
				m_ctrl[i].pos_sp.x += pos_move_rate[0];
				m_ctrl[i].pos_sp.y += pos_move_rate[1];
				m_ctrl[i].pos_sp.z += pos_move_rate[2];
				m_ctrl[i].vel_ff.x = pos_move_rate[0] * m_velff_xy_P;
				m_ctrl[i].vel_ff.y = pos_move_rate[1] * m_velff_xy_P;
				m_ctrl[i].vel_ff.z = pos_move_rate[2] * m_velff_z_P;
				float yaw_move_rate = m_joy[i].axes[2] * 20 * DEG2RAD;
				m_ctrl[i].att_sp.z += yaw_move_rate;

				m_pubPossp[i].publish(m_ctrl[i].pos_sp);
				m_pubVelff[i].publish(m_ctrl[i].vel_ff);
			}
			break;
			case TrjCtrl:{

			}
			break;
			default:
			break;
			}
		}
		//publish
	}
	void joyCallback0(const sensor_msgs::Joy::ConstPtr& joy)
	{
		//0 PosCtrl, 1 AttCtrl
		static bool l_buttons[14];
		
		for(int i=0;i<14;i++){
			m_joy[0].curr_buttons[i] = joy->buttons[i];
			if(m_joy[0].curr_buttons[i] != l_buttons[i])
				m_joy[0].changed_buttons[i] = true;
			else
				;//changed_buttons cleared in iteration
		}
		for(int i=0;i<14;i++){
			l_buttons[i] = m_joy[0].curr_buttons[i];
		}
		m_joy[0].axes[0] = joy->axes[2];//roll
		m_joy[0].axes[1] = joy->axes[5];//pitch
		m_joy[0].axes[2] = joy->axes[1];//yaw
		m_joy[0].axes[3] = joy->axes[0];//thr
	}
	void joyCallback1(const sensor_msgs::Joy::ConstPtr& joy)
	{
		static bool l_buttons[14];
		
		for(int i=0;i<14;i++){
			m_joy[1].curr_buttons[i] = joy->buttons[i];
			if(m_joy[1].curr_buttons[i] != l_buttons[i])
				m_joy[1].changed_buttons[i] = true;
			else
				;//changed_buttons cleared in iteration
		}
		for(int i=0;i<14;i++){
			l_buttons[i] = m_joy[1].curr_buttons[i];
		}
		m_joy[1].axes[0] = joy->axes[2];//roll
		m_joy[1].axes[1] = joy->axes[5];//pitch
		m_joy[1].axes[2] = joy->axes[1];//yaw
		m_joy[1].axes[3] = joy->axes[0];//thr
	}
int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	ros::init(argc, argv, "commander");
	ros::NodeHandle n("~");
	// ros::NodeHandle n;
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);
	Commander commander(n);
	commander.run(50);


  return 0;


}

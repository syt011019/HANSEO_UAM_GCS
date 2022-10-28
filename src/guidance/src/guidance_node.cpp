#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>

#include <uav_status.h>
#include <console_print.h>
#include <uav_manage.h>
#include <guidance.h>

using namespace std;

#define LOOP_HZ 20.0f
#define R2D 180/3.14159265359
#define D2R 3.14159265359/180.0
#define EARTH_RADIUS 6378137.0;

#define dt 0.050;

mavros_msgs::State current_state;
geometry_msgs::Point landing_pad_point;
geometry_msgs::Point human_pose_point;
geometry_msgs::Point human_vel_point;

void callback_state(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}


void callback_human_pose(const geometry_msgs::Point::ConstPtr& msg)
{
	human_pose_point = *msg;
}

void callback_human_vel(const geometry_msgs::Point::ConstPtr& msg)
{
	human_vel_point = *msg;
}

Eigen::Matrix<double, 3, 3> makeCfp(float angle)
{
	Eigen::Matrix<double, 3, 3> C_fp;

	C_fp << cos(angle), 0, -sin(angle),
		 0, 1, 0,
		 sin(angle), 0, cos(angle);

	return C_fp;	
}

Eigen::Matrix<double, 3, 3> makeCpsi(float angle)
{
	Eigen::Matrix<double, 3, 3> C_psi;

	C_psi << cos(angle), sin(angle), 0,
		 -sin(angle), cos(angle), 0,
		 0, 0, 1;

	return C_psi;	
}

unsigned short collisionRecog(Eigen::Vector3d relDist, Eigen::Vector3d relVel, float safeR)
{
	float t_CPA = 0.0;
	float num = 0.0, den = 0.0;
	float r_res = 0.0;
	Eigen::Vector3d temp;

	unsigned short flag = 0;

	num = relDist.dot(relVel);
	den = relVel.dot(relVel);

	t_CPA = -(num / den) * dt;

	if(isnan(t_CPA) != 0)
		t_CPA = 0.0;

	temp = relVel*t_CPA;

	r_res = relDist.norm()*relDist.norm() - temp.norm()*temp.norm();

	if( (r_res < (safeR*safeR)) && t_CPA >= 0 )
		flag = 1;
	else
		flag = 0;
	

	return flag;
}


void ros_start(int argc, char *argv[])
{

	//////////////////////////////////////////////////////////////
	//                       Initializing                       //
	//////////////////////////////////////////////////////////////
	
	ros::init(argc, argv, "precision_landing");
	ros::NodeHandle nh;	
	ros::Rate rate(LOOP_HZ);
	ros::Subscriber human_pose_sub= nh.subscribe<geometry_msgs::Point>("/human_pose_measure", 1, callback_human_pose);
	ros::Subscriber human_vel_sub= nh.subscribe<geometry_msgs::Point>("/human_vel_measure", 1, callback_human_vel);
	uav_manage um;

	double utm_lat2m_deg = 110961.7060516;
	double utm_lon2m_deg = 89476.51124;

	unsigned short caRecog_flag = 0;
	float filterd_velN = 0.0, filterd_velE = 0.0;
	float filterd_velN_CA = 0.0, filterd_velE_CA = 0.0;

	um.add(new uav_status(nh, "/1_", 1));

	ros::Publisher kalman_pad_position_pub = nh.advertise<geometry_msgs::Point>("kalman_pad_position", 10);

	Eigen::Vector3d kalman_pad_position;

	//////////////////////////////////////////////////////////////
	//                   Self Connection Check                  //
	//     Pixhawk Connection Check & ROS Connection Check      //
	//////////////////////////////////////////////////////////////

	while(ros::ok() && !um.uav_list.at(0)->state[0].connected)
	{	
		ros::spinOnce();
		rate.sleep();
	}

	notice("FCC Connection Succeed");
	
	guidance gdc(nh, um.uav_list.at(0));
	
	try
	{

		while(ros::ok())
		{
			um.communication_health_check();
			if( ((int)gdc.gain[20]) == 1 )			
			{

				for(int i = 0; (i < LOOP_HZ)&&ros::ok(); i++)
				{
					ros::spinOnce();

					Eigen::Vector3d landing_pad;
					landing_pad[0] = um.uav_list.at(0)->global[0] + (landing_pad_point.x/utm_lat2m_deg);
					landing_pad[1] = um.uav_list.at(0)->global[1] + (landing_pad_point.y/utm_lon2m_deg);
					landing_pad[2] = landing_pad_point.z;

					Eigen::Vector3d human_pose;
					human_pose[0] = human_pose_point.x;
					human_pose[1] = human_pose_point.y;
					human_pose[2] = human_pose_point.z;

					Eigen::Vector3d human_vel;
					human_vel[0] = human_vel_point.x;
					human_vel[1] = human_vel_point.y;
					human_vel[2] = human_vel_point.z;

					command a = gdc.gen_command(human_pose);						
					
					// ------------- Low Pass Filter --------------- //
					filterd_velN = gdc.gain[14]*filterd_velN + (1-gdc.gain[14])*a.velocity[0];
					filterd_velE = gdc.gain[14]*filterd_velE + (1-gdc.gain[14])*a.velocity[1];
					um.uav_list.at(0)->set_velocity_ned(a.velocity[0], a.velocity[1], a.velocity[2], a.r);
					//um.uav_list.at(0)->set_velocity_ned(filterd_velN, filterd_velE, a.velocity[2], a.r);
					
					//std::cout<<"gain[0] : "<<gdc.gain[0]<<std::endl;
					
					rate.sleep();
				}
			}

		}

	}

	catch(std::domain_error& e){
		error(e.what());
	}
	
}


int main(int argc, char *argv[])
{
	
	try{
		ros_start(argc, argv);
	}
	
	catch(std::domain_error& e)
	{
		error(e.what());
	}
	
	return 0;
}


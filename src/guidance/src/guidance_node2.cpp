#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

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

#define dt 0.5

mavros_msgs::State current_state;
geometry_msgs::Point landing_pad_point;

void callback_state(const mavros_msgs::State::ConstPtr& msg)
{
	
	current_state = *msg;
	
}


void callback_landing_pad(const geometry_msgs::Point::ConstPtr& msg)
{
	landing_pad_point = *msg;
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

	
	/*std::cout << "relative distance: " << relDist << std::endl;
	std::cout << "relative velocity: " << relVel << std::endl;
	std::cout << "temp : " << temp << std::endl;
	std::cout << "temp norm : " << temp.norm() << std::endl;
	std::cout << "relDist norm : " << relDist.norm() << std::endl;
	std::cout << "t_CPA: " << t_CPA << std::endl;
	std::cout << "r_res: " << r_res << std::endl;*/
	

	return flag;
}

Eigen::Vector3d collsionAvoid(Eigen::Vector3d relDist, Eigen::Vector3d UVel, Eigen::Vector3d IVel, float safeR)
{

	float angle_fpLOS = 0.0, angle_psiLOS = 0.0;
	Eigen::Matrix<double, 3, 3> C_fp;
	Eigen::Matrix<double, 3, 3> C_psi;
	Eigen::Matrix<double, 3, 3> C_revfp;
	Eigen::Matrix<double, 3, 3> C_revpsi;

	Eigen::Vector3d UVel_caLOS;
	Eigen::Vector3d relDist_LOS;
	Eigen::Vector3d UVel_curLOS;
	Eigen::Vector3d IVel_LOS;
	Eigen::Vector3d UVel_ca;
	Eigen::Vector3d Att_cmd;

	float beta = 0.0, mu_0 = 0.0;
	float A = 0.0, B = 0.0, C = 0.0;
	float h = 0.0;


	angle_fpLOS = atan2( relDist[2], sqrt(relDist[0]*relDist[0] + relDist[1]*relDist[1]) );
	angle_psiLOS = atan2( relDist[1], relDist[0] );

	C_fp = makeCfp(angle_fpLOS);
	C_psi = makeCpsi(angle_psiLOS);

	relDist_LOS = (C_fp*C_psi)*relDist;
	UVel_curLOS = (C_fp*C_psi)*UVel;
	IVel_LOS = (C_fp*C_psi)*IVel;

	beta = real( sqrt( (relDist_LOS[0]*relDist_LOS[0] - safeR*safeR)/(safeR*safeR) ) );
	mu_0 = atan2( (UVel_curLOS[2] - IVel_LOS[2]), (UVel_curLOS[1] - IVel_LOS[1]) );

	A = 1 + beta*beta;
	B = IVel_LOS[0]*beta + IVel_LOS[1]*cos(mu_0) + IVel_LOS[2]*sin(mu_0);
	C = IVel.norm()*IVel.norm() - UVel.norm()*UVel.norm();

	if(B*B - A*C >= 0)
	{
		h = -B/A + sqrt(B*B -A*C)/A;

		UVel_caLOS[0] = IVel_LOS[0] + beta*h;
		UVel_caLOS[1] = IVel_LOS[1] + h*cos(mu_0);
		UVel_caLOS[2] = IVel_LOS[2] + h*sin(mu_0);

		C_revfp = makeCfp(-angle_fpLOS);
		C_revpsi = makeCpsi(-angle_psiLOS);

		UVel_ca = C_revpsi*C_revfp*UVel_caLOS;
	}
	else
	{
		UVel_ca[0] = UVel[0];
		UVel_ca[1] = UVel[1];
		UVel_ca[2] = UVel[2];
	}

	return UVel_ca;
}

void ros_start(int argc, char *argv[])
{

	//////////////////////////////////////////////////////////////
	//                       Initializing                       //
	//////////////////////////////////////////////////////////////
	
	ros::init(argc, argv, "precision_landing");
    	ros::NodeHandle nh;	
	ros::Rate rate(LOOP_HZ);
	ros::Subscriber landing_pad_sub= nh.subscribe<geometry_msgs::Point>("/pad_landing", 1, callback_landing_pad);
	uav_manage um;

	double utm_lat2m_deg = 110961.7060516;
	double utm_lon2m_deg = 89476.51124;

	unsigned short caRecog_flag = 0;
	float filterd_velN = 0.0, filterd_velE = 0.0;
	float filterd_velN_CA = 0.0, filterd_velE_CA = 0.0;

	um.add(new uav_status(nh, "", 1));

	ros::Publisher boundary_pub = nh.advertise<geometry_msgs::Point>("boundary", 10);
	ros::Publisher kalman_pad_position_pub = nh.advertise<geometry_msgs::Point>("kalman_pad_position", 10);
	ros::Publisher error_pad_position_pub = nh.advertise<geometry_msgs::Point>("error_pad_position", 10);

	geometry_msgs::Point boundary_msg;
	geometry_msgs::Point kalman_pad_position_msg;
	geometry_msgs::Point error_pad_position_msg;
	Eigen::Vector3d boundary;
	Eigen::Vector3d kalman_pad_position;
	Eigen::Vector3d error_pad_position;
////////////////////////////////////////////////////////////////////
	Eigen::Vector3d pad;
	pad << 36.5913019, 126.2951015, 0.0;

////////////////////////////////////////////////////////////////////

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
					landing_pad[0] = um.uav_list.at(0)->global[0] + (landing_pad_point.x / utm_lat2m_deg);
					landing_pad[1] = um.uav_list.at(0)->global[1] + (landing_pad_point.y / utm_lon2m_deg);
					landing_pad[2] = landing_pad_point.z;

					boundary = gdc.lla2meter(um.uav_list.at(0)->global, pad);
					kalman_pad_position = gdc.lla2meter(um.uav_list.at(0)->global, landing_pad);
					error_pad_position = boundary - kalman_pad_position;

					boundary_msg.x = float(0.8);//ref_pad_position[0];
					boundary_msg.y = float(-0.8);//ref_pad_position[1];
					boundary_msg.z = float(1.0);

					kalman_pad_position_msg.x = landing_pad[0];
					kalman_pad_position_msg.y = landing_pad[1];
					kalman_pad_position_msg.z = landing_pad[2];

					error_pad_position_msg.x = error_pad_position[0];
					error_pad_position_msg.y = error_pad_position[1];
					error_pad_position_msg.z = error_pad_position[2];

					boundary_pub.publish(boundary_msg);
					kalman_pad_position_pub.publish(kalman_pad_position_msg);
					error_pad_position_pub.publish(error_pad_position_msg);

					command a = gdc.gen_command(landing_pad);
					//um.uav_list.at(0)->set_velocity_ned(a.velocity[0], a.velocity[1], a.velocity[2], a.r);
					
					// ------------- Low Pass Filter --------------- //
					filterd_velN = gdc.gain[14]*filterd_velN + (1-gdc.gain[14])*a.velocity[0];
					filterd_velE = gdc.gain[14]*filterd_velE + (1-gdc.gain[14])*a.velocity[1];
					um.uav_list.at(0)->set_velocity_ned(a.velocity[0], a.velocity[1], a.velocity[2], a.r);
					
					//std::cout<<"gain[0] : "<<gdc.gain[0]<<std::endl;
					
					rate.sleep();
				}
			}
			
			else if( ((int)gdc.gain[20]) == 2 )
			{
				
				for(int i = 0; (i < LOOP_HZ)&&ros::ok(); i++)
				{
					ros::spinOnce();
					Eigen::Vector3d goal_pos;
					goal_pos[0] = 36.591287;
					goal_pos[1] = 126.295096;
					goal_pos[2] = gdc.gain[17];
					
					Eigen::Vector3d I_pos;
					I_pos[0] = gdc.gain[15];
					I_pos[1] = gdc.gain[16];
					I_pos[2] = gdc.gain[17];
					
					Eigen::Vector3d IVel;
					IVel[0] = 0.0;
					IVel[1] = 0.0;
					IVel[2] = 0.0;

					Eigen::Vector3d U_pos;
					U_pos[0] = um.uav_list.at(0)->global[0];
					U_pos[1] = um.uav_list.at(0)->global[1];
					U_pos[2] = um.uav_list.at(0)->global[2];

					Eigen::Vector3d UVel;
					UVel[0] = um.uav_list.at(0)->velocity_ned[0];
					UVel[1] = um.uav_list.at(0)->velocity_ned[1];
					UVel[2] = um.uav_list.at(0)->velocity_ned[2];

					Eigen::Vector3d UI_relDist;
					UI_relDist[0] = (I_pos[0] - U_pos[0])*utm_lat2m_deg;
					UI_relDist[1] = (I_pos[1] - U_pos[1])*utm_lon2m_deg;
					UI_relDist[2] = (I_pos[2] - U_pos[2]);

					Eigen::Vector3d UI_relVel;
					UI_relVel[0] = IVel[0] - UVel[0];
					UI_relVel[1] = IVel[1] - UVel[1];
					UI_relVel[2] = IVel[2] - UVel[2];

					/*std::cout << "U_pos: " << t_CPA << std::endl;
					std::cout << "r_res: " << r_res << std::endl;
					std::cout << "relative distance: " << relDist << std::endl;
					std::cout << "relative velocity: " << relVel << std::endl;
					std::cout << "safety range: " << safeR << std::endl;*/
					
					caRecog_flag = collisionRecog(UI_relDist, UI_relVel, gdc.gain[18]);
					std::cout<<"recog flag : " << caRecog_flag <<std::endl;
					if(caRecog_flag == 1)
					{
						Eigen::Vector3d UVel_ca;
						UVel_ca = collsionAvoid(UI_relDist, UVel, IVel, gdc.gain[18]);
						UVel_ca[2] = gdc.gain[19]*(goal_pos[2]-U_pos[2]);
						//um.uav_list.at(0)->set_velocity_ned(UVel_ca[0], UVel_ca[1], UVel_ca[2], 0.0);

						// ---------------------- LOW PASS FILTER -------------------//
						filterd_velN_CA = gdc.gain[13]*filterd_velN_CA + (1-gdc.gain[13])*UVel_ca[0];
						filterd_velE_CA = gdc.gain[13]*filterd_velE_CA + (1-gdc.gain[13])*UVel_ca[1];
						um.uav_list.at(0)->set_velocity_ned(filterd_velN_CA, filterd_velE_CA, UVel_ca[2], 0.0);
					}
					else
					{
						command g = gdc.gen_command_goal(goal_pos);
						//um.uav_list.at(0)->set_velocity_ned(g.velocity[0], g.velocity[1], g.velocity[2], 0.0);
					
						// ---------------------- LOW PASS FILTER -------------------//
						filterd_velN_CA = gdc.gain[13]*filterd_velN_CA + (1-gdc.gain[13])*g.velocity[0];
						filterd_velE_CA = gdc.gain[13]*filterd_velE_CA + (1-gdc.gain[13])*g.velocity[1];
						um.uav_list.at(0)->set_velocity_ned(filterd_velN_CA, filterd_velE_CA, g.velocity[2], 0.0);
					}
					
					Eigen::Vector3d UG_relDist;
					UG_relDist[0] = (goal_pos[0] - U_pos[0])*utm_lat2m_deg;
					UG_relDist[1] = (goal_pos[1] - U_pos[1])*utm_lon2m_deg;
					UG_relDist[2] = (goal_pos[2] - U_pos[2]);
		
					if(UG_relDist.norm() <= 2)
					{
						gdc.gain[20] = 1.0;
						std::cout << "Landing mode start!" << std::endl;
						filterd_velN = um.uav_list.at(0)->velocity_ned[0];
						filterd_velE = um.uav_list.at(0)->velocity_ned[1];
					}
					
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


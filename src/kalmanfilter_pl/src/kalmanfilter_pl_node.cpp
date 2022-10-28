#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kalmanfilter.h>
#include <ros/ros.h>

#include <darknet_msg/CNNDetectionList.h>
#include <darknet_msg/CNNDetection.h>
#include <std_msgs/Char.h>
#include <uav_status.h>
#include <geometry_msgs/Point.h>

#include <uav_manage.h>

#include <math.h>	//HJ temp

#define dt 0.05		//1/2.0
#define posQ 1
#define velQ 1
#define posR 10

void callback_cnn(const darknet_msg::CNNDetectionList::ConstPtr& msg);

darknet_msg::CNNDetectionList measure;

int main(int argc, char* argv[])
{
	/////////////////////////////////////////////////
	/////////////////ROS Init////////////////////////
	/////////////////////////////////////////////////

	ros::init(argc, argv, "kalmanfilter_1");
	ros::NodeHandle nh;
	ros::Rate rate(1/dt);
	ros::Subscriber sub_cnn = nh.subscribe<darknet_msg::CNNDetectionList>("image/CNNDetection", 1, callback_cnn);
	ros::Publisher human_pose_pub = nh.advertise<geometry_msgs::Point>("human_pose_measure", 1);
	ros::Publisher human_vel_pub = nh.advertise<geometry_msgs::Point>("human_vel_measure", 1);
	uav_manage um;

	um.add(new uav_status(nh, "/1_", 1));

	darknet_msg::CNNDetectionList detections;
	std_msgs::Char ca_flag;
	ca_flag.data = 0;

	geometry_msgs::Point human_pose_msg;
	geometry_msgs::Point human_vel_msg;

	typedef struct reserved_argument kalman_arg;

	/////////////////////////////////////////////////
	//////////////Kalman Init////////////////////////
	/////////////////////////////////////////////////

	Eigen::Matrix<double, 3, 3> A;
	Eigen::Matrix<double, 3, 3> H;
	Eigen::Matrix<double, 3, 3> Q;
	Eigen::Matrix<double, 3, 3> R;

	Eigen::Matrix<double, 3, 3> P;

	Eigen::Matrix<double, 3, 1> x;
	Eigen::Matrix<double, 3, 1> u;

	Eigen::Matrix<double, 3, 3> K;
	Eigen::Matrix<double, 3, 3> B;

	K << 554.191356, 0, 320.0,
		0, 554.191356, 240.0,
		0, 0, 1;

	Eigen::Matrix<double, 3, 1> z;
	Eigen::Matrix<double, 3, 1> z_human;
	darknet_msg::CNNDetectionList dets;
	darknet_msg::CNNDetectionList dets_empty;

	unsigned int cnt = 0;
	/////////////////////////////////////////////////

	uav_status uav(nh, "/1_", 1);

	Eigen::Quaterniond q;

	Eigen::Matrix<double, 2, 30> Human_time_hist_static;

	for ( int j = 0; j < 30; ++j){
		Human_time_hist_static(0, j) = (dt)*j;
		Human_time_hist_static(1, j) = 1.0;		
	}
	Eigen::MatrixXd Human_time_hist = Human_time_hist_static;
	Eigen::MatrixXd Human_time_hist_pinv = Human_time_hist.completeOrthogonalDecomposition().pseudoInverse();

	//Eigen::Matrix<double, 2, 20> Pad_time_hist_static;


	Eigen::Matrix<double, 3, 1> pixel_vector;
	Eigen::Matrix<double, 3, 1> track_pixel;
	Eigen::Matrix<double, 3, 1> human_vector;
	double Pad_Lat_except[30] = {0.0};
	double Pad_Lon_except[30] = {0.0};
	double Human_Lat_except[30] = {0.0};
	double Human_Lon_except[30] = {0.0};

	float alpha = 0.00;

	Eigen::Matrix3d R_body_to_enu;

	float altitude;
	int label_idx = -1;
	int isSelected = 1;

	int isInitialized = 0;
	int isFirst = 1;
	int human_cnt = 0;

	double utm_lat2m_deg = 110961.7060516;
	double utm_lon2m_deg = 89476.51124;
	double pre_Pad_Lat = 0.0;
	double pre_Pad_Lon = 0.0;
	double Pad_Lat = 0.0;
	double Pad_Lon = 0.0;
	double pre_Human_Lat = 0.0;
	double pre_Human_Lon = 0.0;
	double Pad_Lat_temp[30] = {0.0};
	double Pad_Lon_temp[30] = {0.0};
	double Human_Lat_temp[30] = {0.0};
	double Human_Lon_temp[30] = {0.0};
	int human_find[30] = {0};
	int pad_find[30] = {0};
	double Human_Lat_lpf = 36.123456;
	double Human_Lon_lpf = 126.1234567;
	
	while(ros::ok())
	{
		label_idx = -1;
		dets = measure;
		measure = dets_empty;

		while((!isInitialized)&&(ros::ok()))
		{
			ros::spinOnce();

			////////////////////////////////////////////////////////////////////
			//////////////////////////////Labeling//////////////////////////////
			////////////////////////////////////////////////////////////////////

			int human_find_num = 1;

			for(int i = 0; i < measure.obj.size(); i++)
			{
				if((measure.obj[i].roi.width) > 0.9 && (measure.obj[i].roi.width) < 1.1){					
					human_vector << measure.obj[i].roi.x_offset, measure.obj[i].roi.y_offset, 1.0000;
					human_find_num = 0;					
				}
			}
			//printf("Pad   x : %f, y: %f\n", pixel_vector(0), pixel_vector(1));
			//printf("Human x : %f, y: %f\n", human_vector(0), human_vector(1));

			////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////Detected Data Selection Part//////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////

				altitude = uav.position_ned[2]; 

				q.x() = uav.local_position.pose.orientation.x;
				q.y()  = uav.local_position.pose.orientation.y;
				q.z()  = uav.local_position.pose.orientation.z;
				q.w()  = uav.local_position.pose.orientation.w;

				R_body_to_enu = q.normalized().toRotationMatrix();
				

				//Human
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_human = K.inverse() * human_vector;
		
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_human_flu;
				x_normal_coordinate_human_flu << -x_normal_coordinate_human(1, 0), -x_normal_coordinate_human(0, 0), -x_normal_coordinate_human(2, 0);
		
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_human_enu = R_body_to_enu*x_normal_coordinate_human_flu;
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_human_ned;
				x_normal_coordinate_human_ned << x_normal_coordinate_human_enu(1, 0), x_normal_coordinate_human_enu(0, 0), -x_normal_coordinate_human_enu(2, 0);
				Eigen::Matrix<double, 3, 1> target_ned_human = x_normal_coordinate_human_ned * myAbs(10.0/x_normal_coordinate_human_ned(2,0));	

				Eigen::Matrix<double, 3, 1> target_enu_human = x_normal_coordinate_human_enu * myAbs(10.0/x_normal_coordinate_human_enu(2,0));	

				Eigen::Matrix<double, 3, 1> fcc_based;
				fcc_based = (R_body_to_enu.transpose())*target_enu_human;

				//double Human_Lat = target_ned_human(0, 0); double Human_Lon = target_ned_human(1, 0);

				//printf("target_ned_human(0, 0) :: %f\n", target_ned_human(0, 0));
				//printf("target_ned_human(1, 0) :: %f\n", target_ned_human(1, 0));

				double lat_human_lpf = lat_human_lpf*0.3 + target_ned_human(0, 0)*0.7;
				double lon_human_lpf = lon_human_lpf*0.3 + target_ned_human(1, 0)*0.7;

				double Human_Lat = um.uav_list.at(0)->global[0] + target_ned_human(0, 0)/utm_lat2m_deg; 
				double Human_Lon = um.uav_list.at(0)->global[1] + target_ned_human(1, 0)/utm_lon2m_deg; 
				//double Human_Lat = um.uav_list.at(0)->global[0] + lat_human_lpf/utm_lat2m_deg; 
				//double Human_Lon = um.uav_list.at(0)->global[1] + lon_human_lpf/utm_lon2m_deg; 

				//double Human_Lat = target_ned_human(0, 0); 
				//double Human_Lon = target_ned_human(1, 0); 

			///////////////////////////////////////////////////////////////////////////////
			///////////////////////////exception handling[Human]///////////////////////////
			///////////////////////////////////////////////////////////////////////////////

				for( int j = 0 ; j < 29 ; j++ )
				{
					human_find[j] = human_find[j+1];
				}
				human_find[29] = human_find_num;

				for( int j = 0 ; j < 29 ; j++ )
				{
					Human_Lat_except[j] = Human_Lat_except[j+1];
					Human_Lon_except[j] = Human_Lon_except[j+1];
				}
				Human_Lat_except[29] = Human_Lat;
				Human_Lon_except[29] = Human_Lon;

				//Outlier decision logic
				int except_cnt = 0;
				for (int j = 0; j < 30; j++){
					float x_diff = (Human_Lat_except[j] - Human_Lat)*utm_lat2m_deg;
					float y_diff = (Human_Lon_except[j] - Human_Lon)*utm_lon2m_deg;
					float dist_diff = sqrt(x_diff*x_diff + y_diff*y_diff);

					if( dist_diff > 3.0 ){					
						except_cnt = except_cnt + 1;
					}									
				}

				if(human_vector(0) < 620 && human_vector(1) < 460){
					Human_Lat = Human_Lat;
					Human_Lon = Human_Lon;	
				}
				else{
					Human_Lat = pre_Human_Lat;
					Human_Lon = pre_Human_Lon;	
				}

				
			//////////////////////////////////////////////////////////////////////
			//////////////////////Pre position & Publish//////////////////////////
			//////////////////////////////////////////////////////////////////////
			pre_Human_Lat = Human_Lat;
			pre_Human_Lon = Human_Lon;

			Human_Lat_lpf = Human_Lat;
			Human_Lon_lpf = Human_Lon;

			///////////////////////////////////////////////////////////////
			///////////////////////////Print///////////////////////////////
			///////////////////////////////////////////////////////////////

			//printf("------------------------------------\n");
			//printf("Human Lat    :: %f\n", Human_Lat);
			//printf("Human Lon    :: %f\n", Human_Lon);
			printf("FCC based x :: %f\n", fcc_based(0));
			printf("FCC based y :: %f\n", fcc_based(1));
			printf("====================================\n");
			printf("\n");

			human_pose_msg.x = Human_Lat_lpf;
			human_pose_msg.y = Human_Lon_lpf;
			human_pose_msg.z = altitude;


			human_pose_pub.publish(human_pose_msg);


			rate.sleep();		
		}

	}
	return 0;
}

void callback_cnn(const darknet_msg::CNNDetectionList::ConstPtr& msg)
{
	measure = *msg;
}
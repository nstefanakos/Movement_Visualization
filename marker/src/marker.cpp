# include "ros/ros.h"
# include <sensor_msgs/LaserScan.h>
# include <visualization_msgs/Marker.h>
# include <visualization_msgs/MarkerArray.h>
# include <cmath>
# include <vector>
# include <nav_msgs/Path.h>
# include <marker/cus_mark.h>

using namespace std;

ros::Publisher pub_1;
ros::Publisher pub_2;
ros::Publisher pub_3;
visualization_msgs::MarkerArray marker_array;
geometry_msgs::Point p;
geometry_msgs::PoseStamped point;
visualization_msgs::Marker Marker;
visualization_msgs::Marker mark;
nav_msgs::Path path;

double KALMAN_X(double U)
{
	static const double R = 0.5;
	static const double H = 1.00;
	static double Q = 0.001;
	static double P = 0;
	static double U_old = 0;
	static double K = 0;

	K = P*H/(H*P*H+R);

	U_old = U_old + K*(U-H*U_old);

	P = (1-K*H)*P+Q;

	return U_old;
}

double KALMAN_Y(double U)
{
	static const double R = 1;
	static const double H = 1.00;
	static double Q = 0.001;
	static double P = 0;
	static double U_old = 0;
	static double K = 0;

	K = P*H/(H*P*H+R);

	U_old = U_old + K*(U-H*U_old);

	P = (1-K*H)*P+Q;

	return U_old;
}

int count_1 = 0;
int count_2 = 0;

std::vector<float> old_ranges;
float old_x;
float old_y;
float old_mark_x;
float old_mark_y;
float vel;
float old_vel = 0;
float vel_param;
ros::Time old_time;
double t;



void Callback_1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan new_msg(*msg);

	marker::cus_mark cus_msg;
	
	visualization_msgs::MarkerArray marker_array_2;
	
	
	if (count_2 == 0)
	{
		old_ranges = new_msg.ranges;
		count_2 = count_2 + 1;
	}

	for (int i = 0 ; i < new_msg.ranges.size(); i++)
	{
		if (abs(old_ranges[i] - new_msg.ranges[i]) <= 0.5 || abs(old_ranges[i] - new_msg.ranges[i]) > 10)
		{    
			new_msg.ranges[i] = 0;
		}
	}

	int max = 0;
	int min = new_msg.ranges.size();

	for (int i = 0 ; i < new_msg.ranges.size(); i++)
	{
		if (new_msg.ranges[i] > 0)
		{
			if (max < i)
			{
				max = i;
			}
			if (min > i)
			{
				min = i;
			}
		}
	}
	
		
	float th_1 = new_msg.angle_min + max * new_msg.angle_increment;
	float th_2 = new_msg.angle_min + min * new_msg.angle_increment;
	float x_1 = new_msg.ranges[max] * cos(th_1);
	float x_2 = new_msg.ranges[min] * cos(th_2);
	float y_1 = new_msg.ranges[max] * sin(th_1);
	float y_2 = new_msg.ranges[min] * sin(th_2);
	float x = (x_1 + x_2)/2;
	float y = (y_1 + y_2)/2;
	
	Marker.header.frame_id = "/hokuyo_base_laser_link";
	Marker.header.stamp = ros::Time::now();
	
	Marker.ns = "human";
	Marker.id = count_1;

	Marker.type = visualization_msgs::Marker::ARROW;
	Marker.action = visualization_msgs::Marker::ADD;
	Marker.scale.x = 0.1;
	Marker.scale.y = 0.2;
	Marker.scale.z = 0.2;
	Marker.color.a = 1.0;
	Marker.color.r = 0.0;
	Marker.color.g = 0.0;
	Marker.color.b = 1.0;
	p.x = KALMAN_X(x);
	p.y = KALMAN_Y(y);
	p.z = 0.5;

	mark.header.frame_id = "/hokuyo_base_laser_link";
	mark.header.stamp = ros::Time::now();
	
	mark.ns = "Future";
	mark.id = count_1;

	mark.type = visualization_msgs::Marker::SPHERE;
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.x = 0.0;
	mark.pose.orientation.y = 0.0;
	mark.pose.orientation.z = 0.0;
	mark.pose.orientation.w = 1.0;
	mark.scale.x = 0.2;
	mark.scale.y = 0.2;
	mark.scale.z = 0.2;
	mark.color.a = 1.0;
	mark.color.r = 1.0;
	mark.color.g = 0.0;
	mark.color.b = 0.0;
	mark.pose.position.z = 0.5;
	mark.lifetime = ros::Duration(0.3); // After-image time
	double check_t_now;
	double check_t;
	if (count_1 >= 1)
	{
		ros::Time time_now = msg->header.stamp;
		ros::Duration t_d(time_now - old_time);

		vel = sqrt(pow((p.x - Marker.points[0].x), 2.) + pow((p.y - Marker.points[0].y), 2.))/(t_d.toSec()); 
		vel_param = vel * t ;

		old_time = time_now;
		Marker.points[0].x = old_x;
		Marker.points[0].y = old_y;
	}

	if (p.x > 0)
	{	
		if (count_1%2 != 0)
		{
			old_x = p.x;
			old_y = p.y;

			float slope = (p.y - Marker.points[0].y)/(p.x - Marker.points[0].x);
			float param = (pow(0.5, 2.)/(1 + pow(slope, 2.)));

			int l = (p.x >= Marker.points[0].x) ? 1 : -1;

			p.x = l*sqrt(param) + Marker.points[0].x;
			p.y = l*slope*sqrt(param) + Marker.points[0].y;

			for (float j=0 ; j <= vel_param ; j = j + 0.2)
			{
				float param_2 = (pow((0.5 + j), 2.)/(1 + pow(slope, 2.)));
				mark.id = j;
				mark.pose.position.x = l*sqrt(param_2) + Marker.points[0].x;
				mark.pose.position.y = l*slope*sqrt(param_2) + Marker.points[0].y;
				marker_array_2.markers.push_back(mark);

				point.header = mark.header;
				point.pose.position.x = mark.pose.position.x;
				point.pose.position.y = mark.pose.position.y;
				point.pose.position.z = mark.pose.position.z;
				point.pose.orientation.x = mark.pose.orientation.x;
				point.pose.orientation.y = mark.pose.orientation.y;
				point.pose.orientation.z = mark.pose.orientation.z;
				point.pose.orientation.w = mark.pose.orientation.w;
				path.poses.push_back(point);
			}

			path.header.frame_id = "/hokuyo_base_laser_link";
			path.header.stamp = ros::Time::now();
			pub_3.publish(path);
			
			cus_msg.header.frame_id = "/hokuyo_base_laser_link";
			cus_msg.header.stamp = msg->header.stamp;

			cus_msg.array = marker_array_2;

			pub_2.publish(cus_msg);
		}

		Marker.points.push_back(p);
		Marker.lifetime = ros::Duration(0.1);
		count_1 = count_1 + 1;
	}
			
	if (Marker.points.size() > 1)
	{
		marker_array.markers.push_back(Marker);

		if (marker_array.markers.size() > 1)
		{
			marker_array.markers.erase(marker_array.markers.begin());
		}

		pub_1.publish(marker_array);
		
		Marker.points.erase(Marker.points.begin());
	}
	
	path.poses.erase(path.poses.begin(), path.poses.end());
	marker_array_2.markers.erase(marker_array_2.markers.begin(), marker_array_2.markers.end());
}

	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Human_Marker");

	ros::NodeHandle n;

	n.getParam("/time", t);

	ros::Subscriber sub = n.subscribe("/scan", 1000, Callback_1);

	pub_1 = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
	pub_2 = n.advertise<marker::cus_mark>("visualization_marker_array_2", 1000);
	pub_3 = n.advertise<nav_msgs::Path>("Path_message", 1000);

	ros::spin();

	return 0;
}
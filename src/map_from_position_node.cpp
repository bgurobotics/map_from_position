#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define proportion_size 0.1
#define x_size_of_map 1000
#define y_size_of_map 1000

using namespace cv;
using namespace std;

bool is_first_run = true;
//globals from the odometry node:
float x_global, y_global, theta_global;

float x_cam=x_size_of_map/2, y_cam=y_size_of_map/2, theta_cam;
float x_fus=x_size_of_map/2, y_fus=y_size_of_map/2, theta_fus;
void camera_position_callback(const geometry_msgs::PoseStamped &msg)
{
x_cam=-msg.pose.position.x+proportion_size*x_size_of_map/2;
y_cam=-msg.pose.position.y+proportion_size*y_size_of_map/2;
theta_cam= msg.pose.orientation.z;
	
	
} 



void fusion_position_callback(const geometry_msgs::Pose2D &msg)
{
x_fus=msg.x+proportion_size*(x_size_of_map/2);
y_fus=msg.y+proportion_size*y_size_of_map/2;
theta_fus= msg.theta;	
} 


void update_obstacle_map(cv::Mat &obstacle_map)
{
	
			
				if(	(int)(x_fus/proportion_size)<x_size_of_map && (int)(y_fus/proportion_size)<y_size_of_map)
				{
					obstacle_map.at<uchar>((int)(y_fus/proportion_size),(int)(x_fus/proportion_size))=1;
					//ROS_INFO("x_range[%d]=%f,      y_range[%d]=%f",i,x_range[i],i,y_range[i]);
				}
				else
				{
					obstacle_map.at<uchar>(y_size_of_map-1,x_size_of_map-1)=1;
					
				}
				
				
				
				if(	(int)(x_cam/proportion_size)<x_size_of_map && (int)(y_cam/proportion_size)<y_size_of_map)
				{
					obstacle_map.at<uchar>((int)(y_cam/proportion_size),(int)(x_cam/proportion_size))=127;
					//ROS_INFO("x_range[%d]=%f,      y_range[%d]=%f",i,x_range[i],i,y_range[i]);
				}
				else
				{
					obstacle_map.at<uchar>(y_size_of_map-1,x_size_of_map-1)=127;
					
				}
			
		
	
}


int main(int argc, char** argv){
  ros::init(argc, argv, "map_from_position");
  ros::NodeHandle n;
  ros::Subscriber cam_position_sub = n.subscribe("/mono_odometer/pose", 1000,camera_position_callback);
  ros::Subscriber fusion_position_sub = n.subscribe("/pose2D", 1000,fusion_position_callback);
  ros::Rate loop_rate(30);
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("/rosbot/map", 1);
  sensor_msgs::ImagePtr obstacle_map_msg;
  Mat obstacle_map = cv::Mat(x_size_of_map,y_size_of_map, CV_8UC1,255);
  
  while(n.ok())
  {
	update_obstacle_map(obstacle_map); 
	obstacle_map_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", obstacle_map).toImageMsg();
	pub.publish(obstacle_map_msg);
	loop_rate.sleep();  
	ros::spinOnce();  
  }
}

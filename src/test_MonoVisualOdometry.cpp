#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include "mono_odometry.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>



using namespace std;
using namespace cv;
class vo1 {
public:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher vision_pub;
	ros::Time current_time;
	Mat frame_old,frame,mask;
	cv_bridge::CvImagePtr cv_ptr;
	MonoVisualOdometry::parameters param;
	MonoVisualOdometry odom;
	MonoVisualOdometry::pose position; //output struct

	vo1(){ 
		mask=imread("mask_e.png",0);
		imshow("mask",mask);
		waitKey(1);
		setup();
		odom=MonoVisualOdometry(param);
		set_param();
		sub= n.subscribe("/bottom_camera/image_raw",1,&vo1::message,this);
		vision_pub=n.advertise<nav_msgs::Odometry>("odometry_publisher", 100);
		}	
 
	void setup(){
		param.option.method=1;
		param.option.solver=1;  
        }  
 
	void set_param(){
		odom.nframes=0;
		odom.opticalFlow=true; 
		odom.mask=mask.clone();//imread("/home/rohit/catkin_ws/devel/lib/test_vision/mask_e.png",0);
		}    
        
	void message(const sensor_msgs::ImageConstPtr& raw_image){
		cv_ptr = cv_bridge::toCvCopy(raw_image,sensor_msgs::image_encodings::BGR8);    
		cv_ptr->image.copyTo(frame);  // get new frame from ROS
    
		namedWindow("mesg",1);
		imshow("mesg",frame);
		waitKey(1);
    
		cout<<frame.size()<<"\n";
		  
        odom.nframes++;
		cout<<"count"<<odom.nframes<<"flag"<<odom.opticalFlow<<"\n";
		if(odom.nframes>=2){
			// run odometry
			odom.img1=frame_old.clone();
			odom.img2=frame.clone();
			imshow("img1",odom.img1);
			waitKey(1);
			imshow("img2",odom.img1);
			waitKey(1);
			odom.run(); 	// run the main odometry calculations
			//MonoVisualOdometry::pose position; //output struct
			odom.output(position);  // get output parameters
			/*cout<<"N="<<position.N<<"\n"; // no of good_matches
			cout<<"x_net="<<position.x_net<<"\n"; // net x-translation
			cout<<"y_net="<<position.y_net<<"\n"; // net y-translation
			cout<<"heading_net="<<position.heading_net<<"\n"; // net heading
			cout<<"Z_1="<<position.Z_avg1<<"\n"; // avg Z estimation 1
			cout<<"Z_2="<<position.Z_avg2<<"\n"; // avg Z estimation 2
			cout<<"iters="<<position.iteration<<"\n"; // no of solver iterations
			cout<<"time="<<position.run_time<<"\n"; // .run() time
			cout<<"x_rel="<<position.x_rel<<"\n";  // relative x-translation
			cout<<"y_rel="<<position.y_rel<<"\n";  // relative y-translation
			cout<<"heading_rel="<<position.heading_rel<<"\n";  	  // relative heading change
			cout<<"rot matrix="<<position.rot<<"\n";		//transform estimated using estimateRigidTransform
			cout<<"x_scaled="<<position.x_scaled<<"\n";		// scaled x-translation
			cout<<"y_scaled="<<position.y_scaled<<"\n";		// scaled y-translation
			cout<<"converged error="<<position.error<<"\n";*/
			
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position.heading_rel);
			nav_msgs::Odometry odom_pub;
			current_time = ros::Time::now();
			odom_pub.header.stamp = current_time;
			odom_pub.pose.pose.position.x =position.x_rel;
			odom_pub.pose.pose.position.y =position.y_rel;
			odom_pub.pose.pose.position.z =(position.Z_avg1+position.Z_avg2)/2;
			odom_pub.pose.pose.orientation = odom_quat;
			vision_pub.publish(odom_pub);
		
					}
	
		//copy the frame to frame_old
		frame.copyTo(frame_old);//frame.clone();      	
		//namedWindow("msg",1);
		imshow("msg..",frame_old);
		waitKey(100);
		}
	void spin(){
		ros::spin();
		}
};


int main(int argc, char** argv){  
	ros::init(argc, argv, "vision_node");
	vo1 v;
	v.spin();
	//ros::Publisher vision_pub=n.advertise<nav_msgs::Odometry>("odometry_publisher", 100);
	//ros::Rate loop_rate(10);
	// ROS Plot x_net, y_net, heading_net wrt time
    return 0;
    }

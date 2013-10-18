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
cv_bridge::CvImagePtr cv_ptr;
void message(const sensor_msgs::ImageConstPtr& raw_image)
{
 cv_ptr = cv_bridge::toCvCopy(raw_image,sensor_msgs::image_encodings::BGR8);
}

int main(int argc, char** argv)
{   cv_bridge::CvImagePtr cv_ptr;
	ros::init(argc, argv, "vision_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("image_raw",1, &message);
	ros::Publisher vision_pub=n.advertise<nav_msgs::Odometry>("odometry_publisher", 100);
	//ros::Rate loop_rate(10);
	ros::spin();
	
	
	
    // class input parameters - feature options
    MonoVisualOdometry::parameters param;
/* not needed for optical flow method

    param.option.feature=4;
    param.option.extract=2;
    param.option.match=1;
    param.option.outlier=1;
*/
	param.option.method=2;
    param.option.solver=1; 
        
    Mat frame_old,frame;
    
    frame_old=cv::imread(argv[1]); // for testing
    frame=cv::imread(argv[2]); // for testing
    
    if(frame_old.empty() || frame.empty())
    {
        cout<<"Can't read one of the images\n";
        return -1;
    }
    
    MonoVisualOdometry odom(param);               
    odom.nframes=1; // keeps track of overall frames count
//    odom.nframes=0; // shud be intitialised with =0; for testing =1;
    odom.opticalFlow=true;
    
//    while(!flag){	//actual
    //for(int i=0 ;i<1 ;i++){	// for(;;) testing
    while(ros::ok()){
	// get new frame from ROS
        //cin>>frame; 
        frame=cv_ptr->image;
        odom.nframes++;
       MonoVisualOdometry::pose position; //output struct
       if(odom.nframes>=2) {
  	  // run odometry
  	  odom.img1=frame_old;
  	  odom.img2=frame;
   	  odom.run(); 	// run the main odometry calculations
   	  
	  odom.output(position);  // get output parameters
	  cout<<"N="<<position.N<<"\n"; // no of good_matches
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
          cout<<"converged error="<<position.error<<"\n";
    	}
    	
    	
    //Publishing ros message
    ros::Time current_time;
    current_time = ros::Time::now();
    nav_msgs::Odometry odom_p;
    odom_p.header.stamp = current_time;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position.heading_rel);
    
    odom_p.pose.pose.position.x =position.x_rel;
    odom_p.pose.pose.position.y =position.y_rel;
    odom_p.pose.pose.position.z =(position.Z_avg1+position.Z_avg2)/2;
    odom_p.pose.pose.orientation = odom_quat;
    
    vision_pub.publish(odom_p);

    
	
	//copy the frame to frame_old
	frame_old=frame.clone();    	
    }
    
    // ROS Plot x_net, y_net, heading_net wrt time
    
    return 0;
}

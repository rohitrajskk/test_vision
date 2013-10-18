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

using namespace std;
using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
void message(const sensor_msgs::ImageConstPtr& raw_image)
{
 cv_ptr = cv_bridge::toCvCopy(raw_image,sensor_msgs::image_encodings::BGR8);
}

int main(int argc, char** argv)
{   cv_bridge::CvImagePtr cv_ptr;
	ros::init(argc, argv, "v_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("image_raw",1, &message);
	ros::spin();
	
	
	
    // class input parameters - feature options
    MonoVisualOdometry::parameters param;
/* not needed for optical flow method

    param.option.feature=4;
    param.option.extract=2;
    param.option.match=1;
    param.option.outlier=1;
*/
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
    /*for(int i=0 ;i<1 ;i++ ){	// for(;;) testing
	// get new frame from ROS
        //cin>>frame; 
    //    frame=cv_ptr->image;
        odom.nframes++;

        if(odom.nframes>=2) {
  	  // run odometry
  	  odom.img1=frame_old;
  	  odom.img2=frame;
   	  odom.run(); 	// run the main odometry calculations
   	  MonoVisualOdometry::pose position; //output struct
	  odom.output(position);  // get output parameters
	  cout<<position.N<<"\n"; // no of good_matches
	  cout<<position.x_net<<"\n"; // net x-translation
	  cout<<position.y_net<<"\n"; // net y-translation
	  cout<<position.heading_net<<"\n"; // net heading
	  cout<<position.Z_avg1<<"\n"; // avg Z estimation 1
	  cout<<position.Z_avg2<<"\n"; // avg Z estimation 2
	  cout<<position.iteration<<"\n"; // no of solver iterations
  	  cout<<position.run_time<<"\n"; // .run() time
    	  cout<<position.x_rel<<"\n";  // relative x-translation
    	  cout<<position.y_rel<<"\n";  // relative y-translation
          cout<<position.heading_rel<<"\n";  	  // relative heading change
    	}
	
	//copy the frame to frame_old
	frame_old=frame.clone();    	
    }
    
    // ROS Plot x_net, y_net, heading_net wrt time
    
*/    return 0;
}

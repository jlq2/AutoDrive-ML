#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

 
using namespace std;



class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
   geometry_msgs::Twist driveKeyboard(){
    std::cout << "Type a command and then press enter.  "
      "Use '+' to move forward, 'l' to turn left, "
      "'r' to turn right, '.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd[50];
 

      std::cin.getline(cmd, 50);
      if(cmd[0]!='a' && cmd[0]!='w' && cmd[0]!='d' && cmd[0]!='.')
      {
        std::cout << "unknown command:" << cmd << "\n";

      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      

		//move forward by default
        base_cmd.linear.x = 0.25;
        base_cmd.angular.z = 0;
      

	
      //turn left (yaw) and drive forward at the same time
   	 if(cmd[0]=='a'){
        base_cmd.angular.z = 0.75;
        base_cmd.linear.x = 0.25;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='d'){
        base_cmd.angular.z = -0.75;
        base_cmd.linear.x = 0.25;
      } 
 
      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
    
    return base_cmd;
  }

};



RobotDriver *Gdriver = NULL;
ofstream fs;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {



	static int globalID = 0;
	 
	char buffer [300];

	sprintf(buffer,"/home/juan/VAR-DATASET/OpenCVImages/Image%d.jpg",globalID);

	cv::imwrite(buffer, cv_bridge::toCvShare(msg, "bgr8")->image );
	globalID++;
	 geometry_msgs::Twist cmd = Gdriver->driveKeyboard();
	
	float z = cmd.angular.z;

	fs << buffer << " , " <<  z << endl;


	cv::waitKey(30);



  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{

  fs.open ("/home/juan/VAR-DATASET/data.txt", std::fstream::in | std::fstream::out | std::fstream::app);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
Gdriver = &driver;


  image_transport::ImageTransport it(nh);
 // image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
   image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);


	ros::Rate rate(2.0);
	while(nh.ok())
	{

		ros::spinOnce();
		rate.sleep();

	}


	fs.close();
  ros::spin();
  ros::shutdown();

}

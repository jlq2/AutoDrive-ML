/**
Software License Agreement (BSD)

\file      teleop_node.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat** imagenPendiente = NULL;
bool semaforoEscribir = true;



//callback que se llamara en la subscripcion de image_raw
// se ocupa de recoger esa imagen y almacenarla en el heap, asi depues podran acceder a ella desde otra parte del codigo.
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
	  //Habia otra imagen almacenada? liberamos para pisar en un futuro
	if(*imagenPendiente != NULL){
		free(*imagenPendiente);
	}
		//std::cout << "imagen recibida" << std::endl;
		imagenPendiente[0] =  new cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
		//std::cout << "escrita de forma correcta " << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}



int main(int argc, char *argv[])
{

	cv::namedWindow("view");
	cv::startWindowThread();
	
	// Inicializamos el lugar donde nos guardaremos el puntero a la imagen
	imagenPendiente = (cv::Mat**)malloc(sizeof (cv::Mat));
	*imagenPendiente = 0;
	
	// Creamos nodo
	ros::init(argc, argv, "teleop_twist_joy_node");
	ros::NodeHandle nh(""), nh_param("~");

	// subscripcion a image_raw
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);
	
	// creamos objeto para controlar joystick....
	teleop_twist_joy::TeleopTwistJoy joy_teleop(nh, &nh_param,imagenPendiente);

	ros::spin();
}

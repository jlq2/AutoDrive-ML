#!/usr/bin/env python


##
##  ros::init(argc, argv, "image_listener");
##  ros::NodeHandle nh;
##  RobotDriver driver(nh);
##Gdriver = &driver;


##  image_transport::ImageTransport it(nh);
## image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
##   image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);
#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist

import tensorflow as tf
import scipy.misc
import model
from subprocess import call

sess = tf.InteractiveSession()
saver = tf.train.Saver()
saver.restore(sess, "/home/juan/Escritorio/var/catkin_ws/src/samplePy/scripts/save/model.ckpt")



smoothed_angle = 0



class image_converter:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("robot1/camera/rgb/image_raw",Image,self.callback)
		self.velocity_publisher = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=10)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		vel_msg = Twist()



		cv2.imwrite('/home/juan/VAR-DATASET/using.jpg', cv_image)
		full_image = scipy.misc.imread("/home/juan/VAR-DATASET/using.jpg", mode="RGB")
		image = scipy.misc.imresize(full_image[-150:], [66, 200]) / 255.0
		degrees = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0}, session=sess )[0][0] * 180 / scipy.pi
		#degrees = 45
		vel_msg.angular.z =degrees
		vel_msg.linear.x = 0.25
		call("clear")
		print("Predicted steering angle: " + str(degrees) + " degrees")
		self.velocity_publisher.publish(vel_msg)
		cv2.imshow("Image window", image)
		cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


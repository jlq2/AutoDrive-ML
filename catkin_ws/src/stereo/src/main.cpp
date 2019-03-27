#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

class Stereo {

    private:
        static cv::Mat left;
        static cv::Mat right;
        static cv::Mat out;

        const cv::Mat left_camera_matrix = (cv::Mat_ <float>(3,3) <<    554.845225,  0,          319.731980,
                                                                    0,          554.657211,  239.972478,
                                                                    0,          0,          1.000000);

        const cv::Mat left_distortion = (cv::Mat_<double>(5, 1) << 0.001445, -0.002277, 0.000196, -0.000103, 0.000000);

        const cv::Mat left_rectification = (cv::Mat_<double>(3, 3) <<  0.999995, -0.000334, 0.003204,
                                                                        0.000333, 1.000000, 0.000327,
                                                                        -0.003204, -0.000326, 0.999995);

        const cv::Mat left_projection = (cv::Mat_<float>(3,4)<<    556.073391, 0.000000, 317.288837, 0.000000,
                                                                    0.000000, 556.073391, 239.851135, 0.000000,
                                                                    0.000000, 0.000000, 1.000000, 0.000000);

        const cv::Mat right_camera_matrix = (cv::Mat_ <float>(3,3) <<    554.181216, 0.000000, 320.450171,
                                                                            0.000000, 554.088494, 239.555920,
                                                                            0.000000, 0.000000, 1.000000);

        const cv::Mat right_distortion = (cv::Mat_<double>(5, 1) << 0.000105, -0.001009, 0.000156, 0.000456, 0.000000);

        const cv::Mat right_rectification = (cv::Mat_<double>(3, 3) <<  0.999989, -0.000265, 0.004700,
                                                                        0.000267, 1.000000, -0.000326,
                                                                        -0.004700, 0.000328, 0.999989);

        const cv::Mat right_projection = (cv::Mat_<float>(3,4)<<    556.073391, 0.000000, 317.288837, -480.203223,
                                                                    0.000000, 556.073391, 239.851135, 0.000000,
                                                                    0.000000, 0.000000, 1.000000, 0.000000);

        const cv::Mat rgbIntrinsics = (cv::Mat_<double>(3, 3) <<     5.2161910696979987e+02, 0.,                     3.1755491910920682e+02,
                                                                    0.,                     5.2132946256749767e+02, 2.5921654718027673e+02,
                                                                    0.,                     0.,                     1.);

        const cv::Mat rgbDistortion = (cv::Mat_<double>(5, 1) << 2.5673002693536984e-01, -9.3976085633794137e-01, -1.8605549188751580e-03,
                -2.2232238578189420e-03, 1.2453643444153988e+00);

        const cv::Mat rotationMatrix = (cv::Mat_<double>(3, 3) <<   9.9996518012567637e-01,  2.6765126468950343e-03, -7.9041012313000904e-03,
                                                                    -2.7409311281316700e-03,  9.9996302803027592e-01, -8.1504520778013286e-03,
                                                                    7.8819942130445332e-03,  8.1718328771890631e-03,  9.9993554558014031e-01);

        const cv::Mat translationMatrix = (cv::Mat_<double>(3, 1) << -2.5558943178152542e-02, 1.0109636268061706e-04, 2.0318321729487039e-03);

        const cv::Mat rotation = (cv::Mat_ <float>(3,3) <<      1.0000,         -0.0009,    0.0006,
                                                                0.0009,         0.9999,     0.0134,
                                                                -0.0006,        -0.0134,    0.9999 );

        const cv::Mat transformation=(cv::Mat_<float>(4,4)<<    1.0000,     -0.0009,    0.0006,     -25.36961,
                                                                0.0009,     0.9999,     0.0134,     -1.15194,
                                                                -0.0006,    -0.0134,    0.9999,     -7.98245,
                                                                0,          0,          0,          1);

public:
        Stereo() {

        }

        Stereo(Stereo& s) {

        }

        ~Stereo() {

        }

        static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            try {
                cv::String title("");
                cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
                if (msg->header.frame_id.compare("robot1_tf/openni_camera_link")) {
                    title = "Left camera.";
                    left = image;
                }
                else {
                    title = "Right camera.";
                    right = image;
                }

                cv::imshow(title, image);
                cv::waitKey(30);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
        }

        void generateDepthMap()
        {
            if (!left.empty() && !right.empty())
            {
                cv::Size size(640, 480);
                cv::Mat RL, RR, PL, PR, Q;
                cv::Mat r;
                rotationMatrix.convertTo(r, CV_64F);
                cv::stereoRectify(right_camera_matrix, right_distortion, left_camera_matrix, left_distortion, size, rotationMatrix, translationMatrix, RL, RR, PL, PR, Q);
                cv::Mat LeftMapX, LeftMapY;
                cv::initUndistortRectifyMap(left_camera_matrix, left_distortion, left_rectification, left_projection, size, CV_16SC2, LeftMapX, LeftMapY);
                cv::Mat RightMapX, RightMapY;
                cv::initUndistortRectifyMap(right_camera_matrix, right_distortion, right_rectification, right_projection, size, CV_16SC2, RightMapX, RightMapY);

                cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 15);
                stereo.get()->setMinDisparity(4);
                stereo.get()->setNumDisparities(128);
                stereo.get()->setBlockSize(21);
                stereo.get()->setSpeckleRange(32);
                stereo.get()->setSpeckleWindowSize(80);

                cv::Mat leftFixed, rightFixed;
                cv::remap(left, leftFixed, LeftMapX, LeftMapY, CV_INTER_AREA);
                cv::remap(right, rightFixed, RightMapX, LeftMapY, CV_INTER_AREA);

                cv::Mat leftGray, rightGray;
                cv::cvtColor(leftFixed, leftGray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(rightFixed, rightGray, cv::COLOR_BGR2GRAY);
                stereo.get()->compute(leftGray, rightGray, out);

                cv::imshow("Depth", out * 16);
                cv::waitKey(30);
            }

        }
};

cv::Mat Stereo::left;
cv::Mat Stereo::right;
cv::Mat Stereo::out;

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	//cv::namedWindow("view");
	cv::startWindowThread();
	Stereo s;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber subCam1 = it.subscribe("/robot1/trasera1/trasera1/rgb/image_raw", 1, Stereo::imageCallback);
	image_transport::Subscriber subCam2 = it.subscribe("/robot1/trasera2/trasera2/rgb/image_raw", 1, Stereo::imageCallback);
	ros::Rate rate(10.0);

	while(nh.ok()) {
		ros::spinOnce();
		rate.sleep();
		s.generateDepthMap();
	}

	ros::spin();
	ros::shutdown();
	cv::destroyWindow("view");
}

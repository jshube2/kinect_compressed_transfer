#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

class KinectRGBDownloader
{
	public:
		sensor_msgs::Image rgb;

		KinectRGBDownloader(){
			ros::NodeHandle n;
			image_transport::ImageTransport it(n);
			
			n.getParam("/kinect_rgb_downloader/input_rgb_topic", input_rgb_topic);
			n.getParam("/kinect_rgb_downloader/output_rgb_topic", output_rgb_topic);
			
			rgb_sub = it.subscribe
				(input_rgb_topic, 
				10, &KinectRGBDownloader::rgbCallback, this);

			rgb_pub = n.advertise<sensor_msgs::Image>
				(output_rgb_topic, 10);  

		};


	private:

		ros::Publisher rgb_pub;
		image_transport::Subscriber rgb_sub;

		std::string input_rgb_topic;
		std::string output_rgb_topic;
	
		void rgbCallback(const sensor_msgs::ImageConstPtr& image){
			
			rgb = *image;
			
			rgb_pub.publish(rgb);
		}

};


int main(int argc, char **argv){
  ros::init(argc, argv, "kinect_rgb_downloader");
  KinectRGBDownloader k;
  ros::spin();

  return 0;

}
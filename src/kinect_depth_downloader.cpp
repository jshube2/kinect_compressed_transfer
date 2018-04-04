#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

class KinectDepthDownloader
{
	public:
		sensor_msgs::Image depth;

		KinectDepthDownloader(){
			ros::NodeHandle n;
			image_transport::ImageTransport it(n);

			n.getParam("/kinect_depth_downloader/input_depth_topic", input_depth_topic);
			n.getParam("/kinect_depth_downloader/output_depth_topic", output_depth_topic);
			
			depth_sub = it.subscribe
				(input_depth_topic, 
				10, &KinectDepthDownloader::depthCallback, this);

			depth_pub = n.advertise<sensor_msgs::Image>
				(output_depth_topic, 10);  

		};


	private:

		ros::Publisher depth_pub;
		image_transport::Subscriber depth_sub;

		std::string input_depth_topic;
		std::string output_depth_topic;
	
		void depthCallback(const sensor_msgs::ImageConstPtr& image){
			
			depth = *image;
			
			depth_pub.publish(depth);
		}

};


int main(int argc, char **argv){
  ros::init(argc, argv, "kinect_depth_downloader");
  KinectDepthDownloader k;
  ros::spin();

  return 0;

}
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class KinectSync
{
	public:
		sensor_msgs::Image rgb;
		sensor_msgs::Image depth;
		sensor_msgs::CameraInfo cam;

		bool rgb_lock;
		bool depth_lock;
		bool cam_lock;

		KinectSync(){
			ros::NodeHandle n;

			n.getParam("/kinect_synchronized_publisher/input_rgb_topic", input_rgb_topic);
			n.getParam("/kinect_synchronized_publisher/input_depth_topic", input_depth_topic);
			n.getParam("/kinect_synchronized_publisher/input_camera_topic", input_camera_topic);
			n.getParam("/kinect_synchronized_publisher/output_rgb_topic", output_rgb_topic);
			n.getParam("/kinect_synchronized_publisher/output_depth_topic", output_depth_topic);
			n.getParam("/kinect_synchronized_publisher/output_camera_topic", output_camera_topic);
			
			rgb_sub = n.subscribe<sensor_msgs::Image>
				(input_rgb_topic, 10, 
				&KinectSync::rgbCallback, this);
			depth_sub = n.subscribe<sensor_msgs::Image>
				(input_depth_topic, 
				10, &KinectSync::depthCallback, this);
			cam_sub = n.subscribe<sensor_msgs::CameraInfo>
				(input_camera_topic, 
				10, &KinectSync::camCallback, this);

			rgb_pub = n.advertise<sensor_msgs::Image>
				(output_rgb_topic, 10);  
			depth_pub = n.advertise<sensor_msgs::Image>
				(output_depth_topic, 10);  
			cam_pub = n.advertise<sensor_msgs::CameraInfo>
				(output_camera_topic, 10);

			timer = n.createTimer(ros::Duration(0.1), &KinectSync::syncPub, this);

			rgb_lock = false;
			depth_lock = false;
			cam_lock = false;

			frame_count = 0;
			seq = 0;
			ROS_INFO("Sync constructor complete!");
		};


	private:

		ros::Publisher rgb_pub;
		ros::Publisher depth_pub;
		ros::Publisher cam_pub;
		ros::Subscriber rgb_sub;
		ros::Subscriber depth_sub;
		ros::Subscriber cam_sub;
		ros::Timer timer;
	
		std::string input_rgb_topic, input_depth_topic, input_camera_topic;
		std::string output_rgb_topic, output_depth_topic, output_camera_topic;
		
		int frame_count;
		int seq;
	
		void rgbCallback(const sensor_msgs::ImageConstPtr& image){
			if(rgb_lock){
				return;
			}
				
			rgb = *image;
			rgb_lock = true;
		}

		void depthCallback(const sensor_msgs::ImageConstPtr& image){
			if(depth_lock){
				return;
			}
					
			depth = *image;
			depth_lock = true;
		}

		void camCallback(const sensor_msgs::CameraInfoConstPtr& cam_info){
			if(cam_lock){
				return;
			}
			
			cam = *cam_info;
			cam_lock = true;
		}

		void syncPub(const ros::TimerEvent& event){
			if(!(rgb_lock && depth_lock && cam_lock)){
			   return;
			}
			
			ros::Time current_time = ros::Time::now();
			rgb.header.stamp = current_time;
			depth.header.stamp = current_time;
			cam.header.stamp = current_time;

			rgb.header.seq = seq;
			depth.header.seq = seq;
			cam.header.seq = seq;
			seq++;

			ROS_INFO("Publishing an image pair %d!", seq);	
			rgb_pub.publish(rgb);
			depth_pub.publish(depth);
			cam_pub.publish(cam);

			frame_count = 0;
			rgb_lock = false;
			depth_lock = false;
			cam_lock = false;
		}


};


int main(int argc, char **argv){
  ros::init(argc, argv, "kinect_data_sync_node");
  KinectSync k;
  ros::spin();

  return 0;

}
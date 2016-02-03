#include "../include/detector.h" 

void Detector::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{

  ROS_INFO("Received an Image");
  
}

int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_detector");
  
  /* Do the argc argv thing here if you need to */
  
  NodeHandle nh;
  
  Detector node(&nh);
  spin();
  
  return 0;  
}
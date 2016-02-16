
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/time.h> 

// #include <visp3/gui/vpDisplayGDI.h>
// #include <visp3/gui/vpDisplayOpenCV.h>
// #include <visp3/gui/vpDisplayX.h>
// #include <visp3/io/vpImageIo.h>
// #include <visp3/core/vpIoTools.h>
// #include <visp3/mbt/vpMbEdgeTracker.h>
// #include <visp3/mbt/vpMbEdgeKltTracker.h>
// #include <visp3/io/vpVideoReader.h>
// #include <visp_ros/vpROSGrabber.h>

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class Detector
{
    bool detectionSucces_; // /*!< true if detector detects the object in the image */
    
    string baseImageTopic;
    string maskImageTopic;
    string maskedImageTopic;
    string camInfoTopic;
    string robotPoseTopic;
    
    string camBaseLink;
    string camRGBFrameLink;
    string camRGBOpticalFrameLink;
    string objectLink;
    string robotBaseLink;
    
    NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    Subscriber camInfoSub_,robotPoseSub_;
    Publisher objWorldPub_;
    
    
    int minAreaOfTargetProject;
    double objectRealSurfaceArea;
    double Z_obj_camfrm;
    double X_obj_camfrm;
    double Y_obj_camfrm;
      
    //camera parameters
    double camFx, camFy;
    double camCx, camCy;
    
    //tf broadcaster and transforms
    tf::TransformBroadcaster br;
    tf::Transform tfObCam,tfCamRob,tfRobWorld;//Object in cam frame,Cam in robot frame, Robot in World frame
    tf::TransformListener listenerObjWorld;
    tf::StampedTransform transformObjWorld;
    
    //detected object pose in world frame
    geometry_msgs::PoseStamped poseObjWorld;
  
  public:
    Detector(NodeHandle &_nh, char *detectorType): nh_(_nh), it_(nh_)
    {
      nh_.getParam("base_image_topic", baseImageTopic);
      nh_.getParam("mask_image_topic", maskImageTopic);
      nh_.getParam("segmented_image_topic", maskedImageTopic);
      nh_.getParam("camInfoTopic", camInfoTopic);
      nh_.getParam("minAreaOfTargetProject", minAreaOfTargetProject);
      nh_.getParam("minAreaOfTargetProject", minAreaOfTargetProject);
      nh_.getParam("objectRealSurfaceArea", objectRealSurfaceArea);
      nh_.getParam("camRGBOpticalFrameLink", camRGBOpticalFrameLink);
      nh_.getParam("camBaseLink", camBaseLink);
      nh_.getParam("objectLink", objectLink);
      nh_.getParam("robotBaseLink", robotBaseLink);      
      nh_.getParam("robotPoseTopic", robotPoseTopic);     
      
      ROS_INFO("detector type = %s",detectorType);

	if(strcmp(detectorType,"HISTOGRAM")==0){
	        imageSub_ = it_.subscribe(maskImageTopic, 10, boost::bind(&Detector::segmentationBasedDetection,this, _1));
		ROS_INFO("Detection using purely a color histogram-based segmentation method with image topic = %s",maskImageTopic.c_str());	
	}

	if(strcmp(detectorType,"FEATURE")==0){
	        imageSub_ = it_.subscribe(baseImageTopic, 10, boost::bind(&Detector::featureBasedDetection,this, _1));
		ROS_INFO("Detection using purely a feature-based object detection method");
	}
  
	if(strcmp(detectorType,"COMBINED")==0){
	        imageSub_ = it_.subscribe(baseImageTopic, 10, boost::bind(&Detector::combinedDetection,this, _1));
		ROS_INFO("Detection using a combination of color histogram-based segmentation process and a feature-based object detection method");
	}
		
       // Other subscribers
      camInfoSub_ = nh_.subscribe<sensor_msgs::CameraInfo>(camInfoTopic, 10,boost::bind(&Detector::storeCameraInfo,this,_1));
      
       // Other subscribers
      robotPoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(robotPoseTopic, 10,boost::bind(&Detector::storeLatestRobotPose,this,_1));      
      
      // Publishers
      objWorldPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fixPoint", 1000);
       
       //Setting up fixed transformations if any
      tfCamRob.setOrigin( tf::Vector3(0.0975,0.0,-0.04603));
      tf::Quaternion q_1(0.0,0.38,0.0,0.924);
      tfCamRob.setRotation(q_1);
      

    }
    
    /*! \brief This is a method for detection that outputs the centroid of the largest
     * image blob in the segmented image. Segmented image is obtained using the
     * pal_image_segmentation package.
     */
    void segmentationBasedDetection(const sensor_msgs::Image::ConstPtr&);    
    
    /*! \brief This is a method for detection that outputs the centroid of the object'S
     * projection in the actual image using a feature-based detection method
     */
    void featureBasedDetection(const sensor_msgs::Image::ConstPtr&);    
    
    /*! \brief This is a method for detection that combines both the segmentation and feature
    * -based processes.
    */
    void combinedDetection(const sensor_msgs::Image::ConstPtr&);
    
    /*! \brief This is a method for reading cam info and storing the params into private variables
    * of the class
    */
    void storeCameraInfo(const sensor_msgs::CameraInfo::ConstPtr&);    
    
    /*! \brief This is a method for reading recent robot pose and storing them into private variables
    * of the class
    */
    void storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr&);    
      
    
};
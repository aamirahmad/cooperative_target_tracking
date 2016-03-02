
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
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

#include <pose_cov_ops/pose_cov_ops.h>

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef struct {
   double x,y,z;
} XYZ;

class Stereo_Detector
{
  
    double meanFactor;
    double devFactor;
    
    bool detectionSucces_; // /*!< true if detector detects the object in the image */

    string selfCamInfoTopic;
    string mateCamInfoTopic;
    string selfRobotPoseTopic;
    string mateRobotPoseTopic;
    
    string selfCamBaseLink;
    string selfCamRGBFrameLink;
    string selfCamRGBOpticalFrameLink;
    string selfRobotBaseLink;
    
    string mateCamBaseLink;
    string mateCamRGBFrameLink;
    string mateCamRGBOpticalFrameLink;
    string mateRobotBaseLink;
    
    string detectedObjectLink;
    string projectedObjectLink;    
    string detectedObject_Wframe_Topic;
    string detectedObject_WframeNaive_Topic;

    string P1_cam1_link,P2_cam1_link,Q1_cam2_link;
    string Q1_cam2_naive_link;
    
    
    string selfBlobTopicName;
    Subscriber selfBlobSub_;

    string mateBlobTopicName;
    Subscriber mateBlobSub_;      
    
    NodeHandle nh_;
    Subscriber selfCamInfoSub_,mateCamInfoSub_,blobinImageSub_;
    Publisher objWorldPub_;
    Publisher objWorldPubNaive_;
    
    //camera parameters of self
    double self_camFx, self_camFy;
    double self_camCx, self_camCy;    
    
    //camera parameters of mate
    double mate_camFx, mate_camFy;
    double mate_camCx, mate_camCy;     
    
    
    geometry_msgs::PoseArray selfBlob, mateBlob;
    ros::Time selfMessageTime, mateMessageTime;
    tf::TransformBroadcaster br;
    tf::TransformListener lr;
    tf::StampedTransform genericTransformation;
    
    tf::StampedTransform SelfOptFrameInWorldTransform;
    tf::StampedTransform mateOptFrameInWorldTransform;
    
    
    
    
    
    bool stereoSuccess;
    
  
  
  public:
    Stereo_Detector(NodeHandle &_nh): nh_(_nh)
    {
      nh_.getParam("selfCamInfoTopic", selfCamInfoTopic); 
      nh_.getParam("mateCamInfoTopic", mateCamInfoTopic); 
      nh_.getParam("selfRobotPoseTopic", selfRobotPoseTopic); 
      nh_.getParam("mateRobotPoseTopic", mateRobotPoseTopic); 
      
      nh_.getParam("selfCamBaseLink", selfCamBaseLink); 
      nh_.getParam("selfCamRGBFrameLink", selfCamRGBFrameLink); 
      nh_.getParam("selfCamRGBOpticalFrameLink", selfCamRGBOpticalFrameLink); 
      nh_.getParam("selfRobotBaseLink", selfRobotBaseLink); 
      nh_.getParam("mateCamBaseLink", mateCamBaseLink); 
      nh_.getParam("mateCamRGBFrameLink", mateCamRGBFrameLink); 
      nh_.getParam("mateCamRGBOpticalFrameLink", mateCamRGBOpticalFrameLink); 
      nh_.getParam("mateRobotBaseLink", mateRobotBaseLink); 
      
      nh_.getParam("detectedObjectLink", detectedObjectLink); 
      nh_.getParam("projectedObjectLink", projectedObjectLink); 
      nh_.getParam("detectedObject_Wframe_Topic", detectedObject_Wframe_Topic); 
      nh_.getParam("detectedObject_WframeNaive_Topic", detectedObject_WframeNaive_Topic);
      
      nh_.getParam("P1_cam1", P1_cam1_link); 
      nh_.getParam("P2_cam1", P2_cam1_link); 
      nh_.getParam("Q1_cam2", Q1_cam2_link);       
      nh_.getParam("Q1_cam2_naive", Q1_cam2_naive_link); 
      
      nh_.getParam("selfBlobTopicName", selfBlobTopicName); 
      nh_.getParam("mateBlobTopicName", mateBlobTopicName); 
  
      ROS_INFO("Stereo detector Initiated");

		
       // Other subscribers
      selfCamInfoSub_ = nh_.subscribe<sensor_msgs::CameraInfo>(selfCamInfoTopic, 10,boost::bind(&Stereo_Detector::storeSelfCameraInfo,this,_1));
      
      mateCamInfoSub_ = nh_.subscribe<sensor_msgs::CameraInfo>(mateCamInfoTopic, 10,boost::bind(&Stereo_Detector::storeMateCameraInfo,this,_1));
      
       // Other subscribers
      selfBlobSub_ = nh_.subscribe<geometry_msgs::PoseArray>(selfBlobTopicName, 10,boost::bind(&Stereo_Detector::selfBlobCallBack,this,_1));      
      
      mateBlobSub_ = nh_.subscribe<geometry_msgs::PoseArray>(mateBlobTopicName, 10,boost::bind(&Stereo_Detector::mateBlobCallBack,this,_1));       
      
      objWorldPub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(detectedObject_Wframe_Topic, 1000);
      objWorldPubNaive_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(detectedObject_WframeNaive_Topic, 1000);

    }
    
    /*! \brief This core method that does the main math for stereo matching
     */
    void performStereoMatching();    
      
    /*! \brief This is a method to store the self detected blob
     */
    void selfBlobCallBack(const geometry_msgs::PoseArray::ConstPtr&);    
  
    /*! \brief This is a method to store the mate detected blob
     */
    void mateBlobCallBack(const geometry_msgs::PoseArray::ConstPtr&);    
        
    /*! \brief This is a method for reading self cam info and storing the params into private variables
    * of the class
    */
    void storeSelfCameraInfo(const sensor_msgs::CameraInfo::ConstPtr&);    
    
    /*! \brief This is a method for reading mate cam info and storing the params into private variables
    * of the class
    */
    void storeMateCameraInfo(const sensor_msgs::CameraInfo::ConstPtr&);    
        
    
    
};
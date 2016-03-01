
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

#include <pose_cov_ops/pose_cov_ops.h>

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

class GT_stereo_evaluator
{
  geometry_msgs::PoseWithCovarianceStamped gtPose,
                                           detectedPose_43, detectedPose_44,
					   detectedNaivePose_43, detectedNaivePose_44,
					   fusedPose_43, fusedPose_44,
					   trackedPose_43, trackedPose_44;
					   

  std::vector<double> detectionError_43;   
  std::vector<double> detectionNaiveError_43;   
  std::vector<double> fusedError_43;   
  std::vector<double> trackedError_43;  
  
  std::vector<double> detectionError_44;   
  std::vector<double> detectionNaiveError_44;   
  std::vector<double> fusedError_44;   
  std::vector<double> trackedError_44;   
  

  
  
  Subscriber detected_43,detected_44,
             detectedNaive_43,detectedNaive_44,
	     fused_43,fused_44,
	     tracked_43,tracked_44,
	     gtSub_;
	     
  NodeHandle nh_;	     
  
  
  public:
    GT_stereo_evaluator(NodeHandle &_nh): nh_(_nh)
    {

      gtSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fixedObject", 10,boost::bind(&GT_stereo_evaluator::gt_callback,this,_1));        

      detected_43 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/tgt_stereo_detector_43/detectedObject_Wframe_byRobot_43", 10,boost::bind(&GT_stereo_evaluator::detected_43_callback,this,_1));
      detected_44 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/tgt_stereo_detector_44/detectedObject_Wframe_byRobot_44", 10,boost::bind(&GT_stereo_evaluator::detected_44_callback,this,_1));
      
      detectedNaive_43 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/tgt_stereo_detector_43/detectedObject_WframeNaive_byRobot_43", 10,boost::bind(&GT_stereo_evaluator::detected_43_naive_callback,this,_1));     
      detectedNaive_44 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/tgt_stereo_detector_44/detectedObject_WframeNaive_byRobot_44", 10,boost::bind(&GT_stereo_evaluator::detected_44_naive_callback,this,_1));  

      fused_43 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fusedObject_Wframe_byRobot_43", 10,boost::bind(&GT_stereo_evaluator::fused_43_callback,this,_1));      
      fused_44 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fusedObject_Wframe_byRobot_44", 10,boost::bind(&GT_stereo_evaluator::fused_44_callback,this,_1));    
      
      tracked_43 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/trackedObjectState_byRobot_43", 10,boost::bind(&GT_stereo_evaluator::tracked_43_callback,this,_1));  
      tracked_44 = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/trackedObjectState_byRobot_44", 10,boost::bind(&GT_stereo_evaluator::tracked_44_callback,this,_1));     
      
      bagInitialized=false;

    }
    
    /*! \brief This method stores GT
     */
    void gt_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);    

    /*! \brief This method stores GT
     */
    void detected_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void detected_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void detected_43_naive_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void detected_44_naive_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void fused_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void fused_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);   

    /*! \brief This method stores GT
     */
    void tracked_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);       
    
    /*! \brief This method stores GT
     */
    void tracked_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);       

       
    void finalEvaluation();
    
    ros::Time firstGTtime;
    
    ros::Time currentGTtime,
	      detectedPose_43_time, detectedPose_44_time,
	      detectedNaivePose_43_time, detectedNaivePose_44_time,
	      fusedPose_43_time, fusedPose_44_time,
	      trackedPose_43_time, trackedPose_44_time;      
    bool bagInitialized;
    
};
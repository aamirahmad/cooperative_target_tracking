
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/time.h> 


#include <pose_cov_ops/pose_cov_ops.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;
using namespace Eigen;

class Fusor // one that performs fusion
{
  
  NodeHandle nh_;
  
  // Pay attention that since all is in world frame in fusion class, we drop the Wframe suffixes from the variables
  string selfDetectionTopicName;
  Subscriber selfMeasurementSub_;

  string mateDetectionTopicName;
  Subscriber mateMeasurementSub_;  
  
  string fusedMeasurementTopicName;
  Publisher fuseEstimatePublisher;
 
  geometry_msgs::PoseWithCovarianceStamped selfDetected, mateDetected, fusedMeasurement;
  ros::Time selfMessageTime, mateMessageTime;
  
  double CIcoeff_;
  
  //do this for more than one teammates 
  //vector<string> detectionTopicNames;
  //vector<Subscriber> measurementSubs_;
  
 
  
  public:
    Fusor(NodeHandle &_nh/*, int NumMates*/): nh_(_nh)
    {

      //do this for more than one teammates 
      //detectionTopicNames.resize(NumMates);
      //measurementSubs_.resize(NumMates);
      
      CIcoeff_ = 0.5;

      nh_.getParam("selfDetectionTopicName", selfDetectionTopicName); 
      nh_.getParam("mateDetectionTopicName", mateDetectionTopicName);
      nh_.getParam("fusedMeasurementTopicName", fusedMeasurementTopicName);
      
      //Subscribers
      selfMeasurementSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(selfDetectionTopicName,10,boost::bind(&Fusor::selfMeasurementCallBack,this,_1));
      mateMeasurementSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(mateDetectionTopicName,10,boost::bind(&Fusor::mateMeasurementCallBack,this,_1));
      

      // Publishers
      fuseEstimatePublisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(fusedMeasurementTopicName, 1000);

    }
    
    /*! \brief This is a method for listening to the incoming world-frame measurements 
     * from self detection of target
     */
    void selfMeasurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);    
    
    /*! \brief This is a method for listening to the incoming world-frame measurements 
     * from mates' detection of target
     */
    void mateMeasurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    
    /*! \brief This is a method for doing the real fusion
    */
    void fuseMeasurements();

    
};



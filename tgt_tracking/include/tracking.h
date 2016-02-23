
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
#include <Eigen/Dense>
#include <Eigen/Geometry>

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;
using namespace Eigen;



  
class Tracker // one that performs fusion
{
  
  NodeHandle nh_;
  
  // Pay attention that since all is in world frame in fusion class, we drop the Wframe suffixes from the variables
  string measurementTopicName;
  Subscriber measurementSub_;
  
  
  string trackedStateEstimateTopicName;
  Publisher trackedStateEstimatePublisher;
 
  geometry_msgs::PoseWithCovarianceStamped measurement; // could be obtainedobtained from fusion or only from self in this framework
  ros::Time currentMeasurementMessageTime;
  ros::Time previousMeasurementMessageTime;
  double deltaT; // in seconds previousMeasurementMessageTime - currentMeasurementMessageTime
  
  
  // KF state, Jacobian and noise matrices;
  const int stateSize = 6; // state is position + velocity in 3d Euclidean coordinates
  
  VectorXf prevState;
  VectorXf predictedState;
  VectorXf updatedState;
  
  VectorXf meas;
  VectorXf predictedMeas;
  
  MatrixXf prevCov; // State Covariance
  MatrixXf predCov; // State Covariance
  MatrixXf updatedCov; // State Covariance
  geometry_msgs::PoseWithCovarianceStamped updatedStateToPublish;
  
  MatrixXf G; // Jacobian of measurement function  
  MatrixXf H; // Jacobian of measurement function
  MatrixXf R; // Process noise covariance Matrix
  MatrixXf Q; // Measurement noise covariance Matrix
  
  MatrixXf K; // Kalman Gain Matrix
  MatrixXf I; // Identity Matrix
  
  public:
    Tracker(NodeHandle &_nh/*, int NumMates*/): nh_(_nh)
    {

      currentMeasurementMessageTime = ros::Time::now();
      previousMeasurementMessageTime = currentMeasurementMessageTime;
      
      nh_.getParam("measurementTopicName", measurementTopicName);
      nh_.getParam("trackedStateEstimateTopicName", trackedStateEstimateTopicName);
      
      //Subscribers
      measurementSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(measurementTopicName,10,boost::bind(&Tracker::measurementCallBack,this,_1));
      
      // Publishers
      trackedStateEstimatePublisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(trackedStateEstimateTopicName, 1000);
      
      
      prevState = VectorXf::Zero(stateSize);
      predictedState = VectorXf::Zero(stateSize);
      updatedState = VectorXf::Zero(stateSize);
      
      meas = VectorXf::Zero(stateSize/2);
      predictedMeas = VectorXf::Zero(stateSize/2);
      
      prevCov = MatrixXf::Zero(stateSize, stateSize); // State Covariance
      predCov = MatrixXf::Zero(stateSize, stateSize); // State Covariance
      updatedCov = MatrixXf::Zero(stateSize, stateSize); // State Covariance
      
      G = MatrixXf::Zero(stateSize, stateSize); // Jacobian of measurement function      
      H = MatrixXf::Zero(stateSize/2, stateSize); // Jacobian of measurement function
      R = MatrixXf::Zero(stateSize, stateSize); // Process noise covariance Matrix
      Q = MatrixXf::Zero(stateSize/2, stateSize/2); // Measurement noise covariance Matrix
      
      K = MatrixXf::Zero(stateSize, stateSize/2); // Kalman Gain Matrix
      I = MatrixXf::Zero(stateSize, stateSize); // Identity Matrix      
      
      InitializeJacobianH();
      ConstructNoiseMatrixR();      

    }
    
    /*! \brief This is a method for listening to the incoming fused world-frame measurements 
     * for tracking the object
     */
    void measurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);    
    
    /*! \brief This is a method for doing the KF predict
    */
    void predict();
    
    /*! \brief This is a method for doing the KF update
    */
    void update();
    
    /*! \brief Initializing state
    */
    void InitializeState();    
    
    /*! \brief Initializing Jacobians
    */
    void InitializeJacobianG();
    
    /*! \brief Initializing Jacobians
    */
    void InitializeJacobianH();
    
    /*! \brief Construct Noise Matrices
    */
    void ConstructNoiseMatrixR();
    
    /*! \brief Construct Noise Matrices
    */
    void ConstructNoiseMatrixQ();    
        
    
};



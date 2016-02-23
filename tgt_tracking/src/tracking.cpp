#include "../include/tracking.h" 

void Tracker::InitializeState()
{
 
  prevState << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0;
  predictedState << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0;
  updatedState << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0;
  
  
  prevCov << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
	  , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0;
	  
  predCov << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
	  , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0
	  , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0;	  
	  
  updatedCov  << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
	      , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0
	      , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0
	      , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0
	      , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0
	      , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0;	
	      
  I << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
    , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0;		      
    
}

void Tracker::InitializeJacobianG()
{
 
  G << 1.0 , 0.0 , 0.0 , deltaT , 0.0 , 0.0 
    , 0.0 , 1.0 , 0.0 , 0.0 , deltaT , 0.0
    , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , deltaT
    , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0;

  
}

void Tracker::InitializeJacobianH()
{
 
  H << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
    , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0;
  
}

void Tracker::ConstructNoiseMatrixR()
{
  
  double noisePosVar=0.1, noiseVelVar=0.01;
 
  R << noisePosVar , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 
    , 0.0 , noisePosVar , 0.0 , 0.0 , 0.0 , 0.0
    , 0.0 , 0.0 , noisePosVar , 0.0 , 0.0 , 0.0
    , 0.0 , 0.0 , 0.0 , noiseVelVar , 0.0 , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , noiseVelVar , 0.0
    , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , noiseVelVar;

  
}

void Tracker::ConstructNoiseMatrixQ()
{
  
  Q << measurement.pose.covariance[0] , measurement.pose.covariance[1] , measurement.pose.covariance[2] 
    , measurement.pose.covariance[6] , measurement.pose.covariance[7] , measurement.pose.covariance[8]
    , measurement.pose.covariance[12] , measurement.pose.covariance[13] , measurement.pose.covariance[14];
  
}

void Tracker::predict()
{
  
  predictedState(0) =  prevState(0) + prevState(3)*deltaT;
  predictedState(1) =  prevState(0) + prevState(4)*deltaT;
  predictedState(2) =  prevState(0) + prevState(5)*deltaT;
  predictedState(3) =  prevState(0) ;
  predictedState(4) =  prevState(0) ;
  predictedState(5) =  prevState(0) ;

  predCov = G*prevCov*G.inverse() + R;
  
}

void Tracker::update()
{

 K = predCov*H.transpose()*(H*predCov*H.transpose() + Q).inverse();
 
 meas<<measurement.pose.pose.position.x,measurement.pose.pose.position.y,measurement.pose.pose.position.z;
 predictedMeas<<predictedState(0),predictedState(1),predictedState(2);
 updatedState = predictedState + K*(meas-predictedMeas);
 updatedCov = (I-K*H)*predCov;
 
 updatedStateToPublish.pose.pose.position.x = updatedState[0];
 updatedStateToPublish.pose.pose.position.y = updatedState[1];
 updatedStateToPublish.pose.pose.position.z = updatedState[2];
 updatedStateToPublish.pose.covariance[0] = updatedCov(0);
 updatedStateToPublish.pose.covariance[1] = updatedCov(1);
 updatedStateToPublish.pose.covariance[2] = updatedCov(2);
 updatedStateToPublish.pose.covariance[6] = updatedCov(6);
 updatedStateToPublish.pose.covariance[7] = updatedCov(7);
 updatedStateToPublish.pose.covariance[8] = updatedCov(8);
 updatedStateToPublish.pose.covariance[12] = updatedCov(12);
 updatedStateToPublish.pose.covariance[13] = updatedCov(13);
 updatedStateToPublish.pose.covariance[14] = updatedCov(14); 
 
 trackedStateEstimatePublisher.publish(updatedStateToPublish);
}

void Tracker::measurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
 
  measurement.header = msg->header;
  measurement.pose = msg->pose;
  
  updatedStateToPublish.header = msg->header;
  
  if(previousMeasurementMessageTime == currentMeasurementMessageTime)// This is the first time measurement is obtained
  {      
    currentMeasurementMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
    
    //Only initialize the state
    InitializeState();
    
    return; 
  }
  else
  {
    previousMeasurementMessageTime = currentMeasurementMessageTime;
    currentMeasurementMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
    
    ros::Duration timediff = currentMeasurementMessageTime - previousMeasurementMessageTime;
    deltaT = timediff.toSec();
    
    //just a safety feature
    if(deltaT>0)
    {
      InitializeJacobianG(); // because we have a new deltaT
      
      ConstructNoiseMatrixQ();
      
      predict();
      
      update();        
    }
  }


  
  
}




int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_tracker");
      
  ros::NodeHandle nh;  
  Tracker node(nh);
    
  spin();
  
  return 0;  
}
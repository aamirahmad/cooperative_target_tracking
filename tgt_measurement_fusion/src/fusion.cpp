#include "../include/fusion.h" 


void Fusor::fuseMeasurements()
{

  // Check if measurements are not far apart, if so choose the latest;
  ros::Duration timediff = selfMessageTime - mateMessageTime;
  
  double thresholdTimeInSec = 0.2,timediffInSec = timediff.toSec();

  if(timediffInSec > thresholdTimeInSec)
  {
    //self message is much newer. Simply use this  
    fusedMeasurement.pose = selfDetected.pose;
    fusedMeasurement.header = selfDetected.header; 
    
    fuseEstimatePublisher.publish(fusedMeasurement);
  }
  
  if(timediffInSec < -thresholdTimeInSec)
  {
    //mate message is much newer. Simply use this
    fusedMeasurement.pose = mateDetected.pose;
    fusedMeasurement.header = mateDetected.header;  
    
    fuseEstimatePublisher.publish(fusedMeasurement);
  }
  
  if(timediffInSec > -thresholdTimeInSec && timediffInSec < thresholdTimeInSec)
  {
    //messages from self and teammate are close enough to perform CI
    
    //ROS_INFO("Fusing");
    
    Vector3f a,b,c;
    a << selfDetected.pose.pose.position.x, selfDetected.pose.pose.position.y, selfDetected.pose.pose.position.z;
    b << mateDetected.pose.pose.position.x, mateDetected.pose.pose.position.y, mateDetected.pose.pose.position.z;
    
    Matrix3f A,B,C,A_in,B_in,C_in;
    A << selfDetected.pose.covariance[0], selfDetected.pose.covariance[1], selfDetected.pose.covariance[2],
        selfDetected.pose.covariance[6], selfDetected.pose.covariance[7], selfDetected.pose.covariance[8],
        selfDetected.pose.covariance[12],selfDetected.pose.covariance[13], selfDetected.pose.covariance[14];
    
    B << mateDetected.pose.covariance[0], mateDetected.pose.covariance[1], mateDetected.pose.covariance[2],
        mateDetected.pose.covariance[6], mateDetected.pose.covariance[7], mateDetected.pose.covariance[8],
        mateDetected.pose.covariance[12],mateDetected.pose.covariance[13], mateDetected.pose.covariance[14];
    
    A_in = A.inverse();
    B_in = B.inverse();
    C_in = CIcoeff_*A_in + (1-CIcoeff_)*B_in;
    C = C_in.inverse();
    c = C*(CIcoeff_*A_in*a + (1-CIcoeff_)*B_in*b);
    
    //just initializing with the selfDetectedPose
    fusedMeasurement.pose = selfDetected.pose;
    
    fusedMeasurement.pose.pose.position.x = c[0]; 
    fusedMeasurement.pose.pose.position.y = c[1]; 
    fusedMeasurement.pose.pose.position.z = c[2];
    fusedMeasurement.pose.covariance[0] = C(0);
    fusedMeasurement.pose.covariance[1] = C(1);
    fusedMeasurement.pose.covariance[2] = C(2);
    fusedMeasurement.pose.covariance[6] = C(3);
    fusedMeasurement.pose.covariance[7] = C(4);
    fusedMeasurement.pose.covariance[8] = C(5);
    fusedMeasurement.pose.covariance[12] = C(6);
    fusedMeasurement.pose.covariance[13] = C(7);
    fusedMeasurement.pose.covariance[14] = C(8);
    
    if(timediffInSec>0)
      fusedMeasurement.header = selfDetected.header; 
    else
      fusedMeasurement.header = mateDetected.header;    
    
    fuseEstimatePublisher.publish(fusedMeasurement);
  }
    
  
}



void Fusor::mateMeasurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  //ROS_INFO("Received a teammate measurement Image");
  mateDetected.pose = msg->pose;
  mateDetected.header = msg->header;
  mateMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  fuseMeasurements();
  
}



void Fusor::selfMeasurementCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  //ROS_INFO("Received a self measurement Image");
  selfDetected.pose = msg->pose;
  selfDetected.header = msg->header;
  selfMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  fuseMeasurements();
  
}


int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_fusor");

  ros::NodeHandle nh;  
  Fusor node(nh);
    
  spin();
  
  return 0;  
}
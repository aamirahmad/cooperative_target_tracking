#include "../include/gt_vs_stereo.h" 



void GT_stereo_evaluator::gt_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

  gtPose.header = msg->header;
  
  gtPose.pose.pose.position.x = msg->pose.pose.position.x;
  gtPose.pose.pose.position.y = -msg->pose.pose.position.y;
  gtPose.pose.pose.position.z = -msg->pose.pose.position.z;
  
  currentGTtime = msg->header.stamp;
  
  if(bagInitialized==false)
  {
    firstGTtime = msg->header.stamp;
  }
  
  bagInitialized = true;
}


void GT_stereo_evaluator::detected_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  detectedPose_43.header = msg->header;
  detectedPose_43.pose = msg->pose;  
  
  detectedPose_43_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = detectedPose_43.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = detectedPose_43.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = detectedPose_43.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  detectionError_43.push_back(error);  
  
}


void GT_stereo_evaluator::detected_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){   
  
  detectedPose_44.header = msg->header;
  detectedPose_44.pose = msg->pose;  
  
  detectedPose_44_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = detectedPose_44.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = detectedPose_44.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = detectedPose_44.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  
//   cout << "gtPose.pose.pose.position.x = " <<gtPose.pose.pose.position.x<< endl;
//   cout << "gtPose.pose.pose.position.y = " <<gtPose.pose.pose.position.y<< endl;
//   cout << "gtPose.pose.pose.position.z = " <<gtPose.pose.pose.position.z<< endl;
//   
//   cout << "detectedPose_44.pose.pose.position.x = " <<detectedPose_44.pose.pose.position.x<< endl;
//   cout << "detectedPose_44.pose.pose.position.y = " <<detectedPose_44.pose.pose.position.y<< endl;
//   cout << "detectedPose_44.pose.pose.position.z = " <<detectedPose_44.pose.pose.position.z<< endl;
//   
//   cout << "error = " <<error<< endl;
  
  detectionError_44.push_back(error);
  
}


void GT_stereo_evaluator::detected_43_naive_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  detectedNaivePose_43.header = msg->header;
  detectedNaivePose_43.pose = msg->pose;  
  
  detectedNaivePose_43_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = detectedNaivePose_43.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = detectedNaivePose_43.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = detectedNaivePose_43.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  detectionNaiveError_43.push_back(error);  
  
}


void GT_stereo_evaluator::detected_44_naive_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  detectedNaivePose_44.header = msg->header;
  detectedNaivePose_44.pose = msg->pose;  
  
  detectedNaivePose_44_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = detectedNaivePose_44.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = detectedNaivePose_44.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = detectedNaivePose_44.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  detectionNaiveError_44.push_back(error);  
  
}


void GT_stereo_evaluator::fused_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  fusedPose_43.header = msg->header;
  fusedPose_43.pose = msg->pose;  
  
  fusedPose_43_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = fusedPose_43.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = fusedPose_43.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = fusedPose_43.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  fusedError_43.push_back(error);  
  
}


void GT_stereo_evaluator::fused_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  fusedPose_44.header = msg->header;
  fusedPose_44.pose = msg->pose;  
  
  fusedPose_44_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = fusedPose_44.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = fusedPose_44.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = fusedPose_44.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  fusedError_44.push_back(error);  
  
}  


void GT_stereo_evaluator::tracked_43_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  trackedPose_43.header = msg->header;
  trackedPose_43.pose = msg->pose;  
  
  trackedPose_43_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = trackedPose_43.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = trackedPose_43.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = trackedPose_43.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  trackedError_43.push_back(error);  
  
}

void GT_stereo_evaluator::tracked_44_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
 
  trackedPose_44.header = msg->header;
  trackedPose_44.pose = msg->pose;  
  
  trackedPose_44_time = msg->header.stamp;
  
  double xEr,yEr,zEr,error;
  xEr = trackedPose_44.pose.pose.position.x - gtPose.pose.pose.position.x;
  yEr = trackedPose_44.pose.pose.position.y - gtPose.pose.pose.position.y;
  zEr = trackedPose_44.pose.pose.position.z - gtPose.pose.pose.position.z;
  
  error = pow((xEr*xEr+yEr*yEr+zEr*zEr),0.5);
  trackedError_44.push_back(error);    
}


void GT_stereo_evaluator::finalEvaluation(){
  
  ROS_INFO("Reached the end of dataset. Starting final evaluation now");
  
  double sum = std::accumulate(detectionError_44.begin(), detectionError_44.end(), 0.0);
  double mean = sum / detectionError_44.size();

  double sq_sum = std::inner_product(detectionError_44.begin(), detectionError_44.end(), detectionError_44.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / detectionError_44.size() - mean * mean);
  
  cout << mean << endl;
  cout << stdev << endl;  
  
  
  sum = std::accumulate(trackedError_43.begin(), trackedError_43.end(), 0.0);
  mean = sum / trackedError_43.size();

  sq_sum = std::inner_product(trackedError_43.begin(), trackedError_43.end(), trackedError_43.begin(), 0.0);
  stdev = std::sqrt(sq_sum / trackedError_43.size() - mean * mean);
  
  cout << mean << endl;
  cout << stdev << endl;    
  
}











int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_stereo_evaluator");

//   if (argc < 2)
//     {
//       ROS_WARN("WARNING: say something here");
//       return 1;
//     }
//   else
//   {
//     ROS_INFO("Detector provided = %s for %s",argv[1]);
//   }
      
  ros::NodeHandle nh("~");  
  GT_stereo_evaluator node(nh);
    
  ros::Rate looprate(120);
  while(ros::ok())
  {
    spinOnce();
    
    ros::Duration timediff = node.currentGTtime - node.firstGTtime;
    //ROS_INFO("timediff in seconds = %f",timediff.toSec());
    
    if(fabs(timediff.toSec()) > 360.0)
    {
      node.finalEvaluation();
      ros::shutdown();
    }
    
  }
  
//   spin();
  
  return 0;  
}
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
  
  
  if(error>2)
  {
    cout << "error = " <<error<< endl;
    return;
  }  
  
  
  if(error!=error){
    //cout << "error = " <<error<< endl;
    //cout << "gtPose.pose.pose.position.x = " <<gtPose.pose.pose.position.x<< endl;
    //cout << "gtPose.pose.pose.position.y = " <<gtPose.pose.pose.position.y<< endl;
    //cout << "gtPose.pose.pose.position.z = " <<gtPose.pose.pose.position.z<< endl;
    
    //cout << "fusedPose_43.pose.pose.position.x = " <<fusedPose_43.pose.pose.position.x<< endl;
    //cout << "fusedPose_43.pose.pose.position.y = " <<fusedPose_43.pose.pose.position.y<< endl;
    //cout << "fusedPose_43.pose.pose.position.z = " <<fusedPose_43.pose.pose.position.z<< endl;    
    
  }
  else
  {  
    fusedError_43.push_back(error);  
    fprintf(resultsFile43,"%d\t%f\t%f\t%f\t%f\t%f\t%f\n",fusedPose_43.header.seq,fusedPose_43.pose.pose.position.x,fusedPose_43.pose.pose.position.y, fusedPose_43.pose.pose.position.z, gtPose.pose.pose.position.x, gtPose.pose.pose.position.y, gtPose.pose.pose.position.z);
  }
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
  
  if(error>2)
  {
    cout << "error = " <<error<< endl;
    return;
  }
  
  if(error!=error){
    //cout << "error = " <<error<< endl;
    //cout << "gtPose.pose.pose.position.x = " <<gtPose.pose.pose.position.x<< endl;
    //cout << "gtPose.pose.pose.position.y = " <<gtPose.pose.pose.position.y<< endl;
    //cout << "gtPose.pose.pose.position.z = " <<gtPose.pose.pose.position.z<< endl;
    
    //cout << "fusedPose_44.pose.pose.position.x = " <<fusedPose_44.pose.pose.position.x<< endl;
    //cout << "fusedPose_44.pose.pose.position.y = " <<fusedPose_44.pose.pose.position.y<< endl;
    //cout << "fusedPose_44.pose.pose.position.z = " <<fusedPose_44.pose.pose.position.z<< endl;
  }
  else
  {
    fusedError_44.push_back(error);  
    fprintf(resultsFile44,"%d\t%f\t%f\t%f\t%f\t%f\t%f\n",fusedPose_44.header.seq, fusedPose_44.pose.pose.position.x,fusedPose_44.pose.pose.position.y, fusedPose_44.pose.pose.position.z, gtPose.pose.pose.position.x, gtPose.pose.pose.position.y, gtPose.pose.pose.position.z);
  }
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
  
  double sum = std::accumulate(fusedError_43.begin(), fusedError_43.end(), 0.0);
  double mean = sum / fusedError_43.size();

  double sq_sum = std::inner_product(fusedError_43.begin(), fusedError_43.end(), fusedError_43.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / fusedError_43.size() - mean * mean);
  
  cout << "fusedError_43 MEAN = "<< mean << endl;
  cout << "fusedError_43 STDV = "<< stdev << endl;  
  
  
  sum = std::accumulate(fusedError_44.begin(), fusedError_44.end(), 0.0);
  mean = sum / fusedError_44.size();

  sq_sum = std::inner_product(fusedError_44.begin(), fusedError_44.end(), fusedError_44.begin(), 0.0);
  stdev = std::sqrt(sq_sum / fusedError_44.size() - mean * mean);
  
  cout << "fusedError_44 MEAN = "<<  mean << endl;
  cout << "fusedError_44 STDV = "<< stdev << endl;   
  
  fcloseall();
  
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
    
    if(fabs(timediff.toSec()) > 580.0)
    {
      node.finalEvaluation();
      ros::shutdown();
    }
    
  }
  
//   spin();
  
  return 0;  
}
#include "../include/stereo_detector.h" 

int LineLineIntersect(XYZ p1,XYZ p2,XYZ p3,XYZ p4,XYZ *pa,XYZ *pb,
   double *mua, double *mub)
{
   XYZ p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;
   double EPS = 0.0001;

   p13.x = p1.x - p3.x;
   p13.y = p1.y - p3.y;
   p13.z = p1.z - p3.z;
   p43.x = p4.x - p3.x;
   p43.y = p4.y - p3.y;
   p43.z = p4.z - p3.z;
   if (fabs(p43.x) < EPS && fabs(p43.y) < EPS && fabs(p43.z) < EPS)
      return(FALSE);
   p21.x = p2.x - p1.x;
   p21.y = p2.y - p1.y;
   p21.z = p2.z - p1.z;
   if (fabs(p21.x) < EPS && fabs(p21.y) < EPS && fabs(p21.z) < EPS)
      return(FALSE);

   d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
   d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
   d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
   d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
   d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

   denom = d2121 * d4343 - d4321 * d4321;
   if (fabs(denom) < EPS)
      return(FALSE);
   numer = d1343 * d4321 - d1321 * d4343;

   *mua = numer / denom;
   *mub = (d1343 + d4321 * (*mua)) / d4343;

   pa->x = p1.x + *mua * p21.x;
   pa->y = p1.y + *mua * p21.y;
   pa->z = p1.z + *mua * p21.z;
   pb->x = p3.x + *mub * p43.x;
   pb->y = p3.y + *mub * p43.y;
   pb->z = p3.z + *mub * p43.z;

   return(TRUE);
}

void Stereo_Detector::storeSelfCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  self_camFx = msg->K[0];
  self_camFy = msg->K[4];
  
  self_camCx = msg->K[2];
  self_camCy = msg->K[5];
  
  ROS_INFO("self cameraFx = %f",self_camFx);
  ROS_INFO("self cameraFy = %f",self_camFy);
  
  //storing these parameters only once is required 
  selfCamInfoSub_.shutdown();
}


void Stereo_Detector::performStereoMatching()
{

  // Check if measurements are not far apart, if so choose the latest;
  ros::Duration timediff = selfMessageTime - mateMessageTime;
  
  double thresholdTimeInSec = 0.2,timediffInSec = timediff.toSec();

  if(timediffInSec > thresholdTimeInSec)
  {
    //self message is much newer. Simply use this  
    return;
  }
  
  if(timediffInSec < -thresholdTimeInSec)
  {
    //mate message is much newer. Simply use this
    return;
  }
  
  if(timediffInSec > -thresholdTimeInSec && timediffInSec < thresholdTimeInSec)
  {

    //ROS_INFO("performStereoMatching");
    //perform stereo matching because we have some coherence in time
    
    
    try
    {
      //lr.waitForTransform(mateCamRGBOpticalFrameLink, P1_cam1_link,mateMessageTime, ros::Duration(3.0));
      lr.lookupTransform("/world_link", selfCamRGBOpticalFrameLink, ros::Time(0), SelfOptFrameInWorldTransform);
    }      
    
    catch (tf::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      return;
    }  
    try
    {
      lr.lookupTransform("/world_link", mateCamRGBOpticalFrameLink, ros::Time(0), mateOptFrameInWorldTransform);
    }      
    
    catch (tf::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      return;
    }      
    
    //see the technical report for the variable notations.
    geometry_msgs::Pose P1_cam1, P2_cam1,
                        P1_cam2, P2_cam2,
			p1_cam1, p2_cam1,
			p1_cam2, p2_cam2;
			
    p1_cam1.position.x = selfBlob.poses[0].position.x - self_camCx;
    p1_cam1.position.y = selfBlob.poses[0].position.y - self_camCy;

    //p2_cam1 is basically same as p1_cam1
    
    //Set up the points P1_cam1, P2_cam1;
    P1_cam1.position.x = 1.0*p1_cam1.position.x/(self_camFx);
    P1_cam1.position.y = 1.0*p1_cam1.position.y/(self_camFy);
    P1_cam1.position.z = 1.0; // in meter
    P2_cam1.position.x = 2.0*p1_cam1.position.x/(self_camFx);
    P2_cam1.position.y = 2.0*p1_cam1.position.y/(self_camFy);
    P2_cam1.position.z = 2.0; // in meter    
    
    tf::Transform transform1;
    
    transform1.setOrigin(tf::Vector3(P1_cam1.position.x, P1_cam1.position.y, P1_cam1.position.z));
    tf::Quaternion q1(0,0,0,1);
    transform1.setRotation(q1);
	
    br.sendTransform(tf::StampedTransform(transform1, selfMessageTime, selfCamRGBOpticalFrameLink, P1_cam1_link));
    
    transform1.setOrigin(tf::Vector3(P2_cam1.position.x, P2_cam1.position.y, P2_cam1.position.z));
    
    br.sendTransform(tf::StampedTransform(transform1, selfMessageTime, selfCamRGBOpticalFrameLink, P2_cam1_link));

	try
	{
	 //lr.waitForTransform(mateCamRGBOpticalFrameLink, P1_cam1_link,mateMessageTime, ros::Duration(3.0));
	  lr.lookupTransform(mateCamRGBOpticalFrameLink, P1_cam1_link, ros::Time(0), genericTransformation);

	  P1_cam2.position.x = genericTransformation.getOrigin().x();
	  P1_cam2.position.y = genericTransformation.getOrigin().y();
	  P1_cam2.position.z = genericTransformation.getOrigin().z();
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	}          
	try
	{
	  //lr.waitForTransform(mateCamRGBOpticalFrameLink, P2_cam1_link, mateMessageTime, ros::Duration(3.0));
	  lr.lookupTransform(mateCamRGBOpticalFrameLink, P2_cam1_link, ros::Time(0), genericTransformation);

	  P2_cam2.position.x = genericTransformation.getOrigin().x();
	  P2_cam2.position.y = genericTransformation.getOrigin().y();
	  P2_cam2.position.z = genericTransformation.getOrigin().z();
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	}
	
    p1_cam2.position.x = mate_camFx*(P1_cam2.position.x)/(P1_cam2.position.z);
    p1_cam2.position.y = mate_camFy*(P1_cam2.position.y)/(P1_cam2.position.z);
    
    p2_cam2.position.x = mate_camFx*(P2_cam2.position.x)/(P2_cam2.position.z);
    p2_cam2.position.y = mate_camFy*(P2_cam2.position.y)/(P2_cam2.position.z);   
    
    
    // Now find the closest point on the line made by  p1_cam2 and p2_cam2 from the mateBlob.poses[0], i.e. from the center of the mate blob
    
    geometry_msgs::Pose p3_cam2, p_closest_cam2, Q1_cam2, Q1_cam2_naive;
    
    p3_cam2.position.x = mateBlob.poses[0].position.x - mate_camCx;
    p3_cam2.position.y = mateBlob.poses[0].position.y - mate_camCy;    
    
    double a = (p2_cam2.position.x - p1_cam2.position.x)/(p2_cam2.position.y - p1_cam2.position.y);
    
    p_closest_cam2.position.x = (p3_cam2.position.y - p1_cam2.position.y + a*p3_cam2.position.x + p1_cam2.position.x/a)/(a+(1/a));
    p_closest_cam2.position.y = p3_cam2.position.y - a*(p_closest_cam2.position.x - p3_cam2.position.x);
    
    double distClosestPointFromBlobCenter=0.0;
    distClosestPointFromBlobCenter = pow((pow(p_closest_cam2.position.x-p3_cam2.position.x,2)+pow(p_closest_cam2.position.y-p3_cam2.position.y,2)),0.5);
    
    Q1_cam2.position.x = 1.0*p_closest_cam2.position.x/(mate_camFx);
    Q1_cam2.position.y = 1.0*p_closest_cam2.position.y/(mate_camFy);
    Q1_cam2.position.z = 1.0;
    transform1.setOrigin(tf::Vector3(Q1_cam2.position.x, Q1_cam2.position.y, Q1_cam2.position.z));
    br.sendTransform(tf::StampedTransform(transform1, selfMessageTime, mateCamRGBOpticalFrameLink, Q1_cam2_link));    
   
     ///@TODO this is only to compare a baseline naive approach
    Q1_cam2_naive.position.x = 1.0*p3_cam2.position.x/(mate_camFx);
    Q1_cam2_naive.position.y = 1.0*p3_cam2.position.y/(mate_camFy);
    Q1_cam2_naive.position.z = 1.0;
    transform1.setOrigin(tf::Vector3(Q1_cam2_naive.position.x, Q1_cam2_naive.position.y, Q1_cam2_naive.position.z));
    br.sendTransform(tf::StampedTransform(transform1, selfMessageTime, mateCamRGBOpticalFrameLink, Q1_cam2_naive_link));     
    
    
   
    
    
    
    //ROS_INFO("p_closest_cam2.position.x = %f",p_closest_cam2.position.x);
    //ROS_INFO("p_closest_cam2.position.y = %f",p_closest_cam2.position.y);
    
    //now prepare to find the actual 3D point.
    XYZ P1,P2,P3,P4,P4_naive,Pa,Pb,Pa_naive,Pb_naive; double mua,mub,muaNaive,mubNaive;

	  P1.x = SelfOptFrameInWorldTransform.getOrigin().x();
	  P1.y = SelfOptFrameInWorldTransform.getOrigin().y();
	  P1.z = SelfOptFrameInWorldTransform.getOrigin().z();
	  
	  P3.x = mateOptFrameInWorldTransform.getOrigin().x();
	  P3.y = mateOptFrameInWorldTransform.getOrigin().y();
	  P3.z = mateOptFrameInWorldTransform.getOrigin().z();

	
	try
	{
	  //lr.waitForTransform("world_link", P2_cam1_link, selfMessageTime,  ros::Duration(3.0));
	  lr.lookupTransform("world_link", P2_cam1_link, ros::Time(0), genericTransformation);

	  P2.x = genericTransformation.getOrigin().x();
	  P2.y = genericTransformation.getOrigin().y();
	  P2.z = genericTransformation.getOrigin().z();
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	} 

	try
	{
	  //lr.waitForTransform("world_link", Q1_cam2_link, mateMessageTime, ros::Duration(3.0));
	  lr.lookupTransform("world_link", Q1_cam2_link, ros::Time(0), genericTransformation);

	  P4.x = genericTransformation.getOrigin().x();
	  P4.y = genericTransformation.getOrigin().y();
	  P4.z = genericTransformation.getOrigin().z();
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	}
	try
	{
	  //lr.waitForTransform("world_link", Q1_cam2_naive_link, mateMessageTime, ros::Duration(3.0));
	  lr.lookupTransform("world_link", Q1_cam2_naive_link, ros::Time(0), genericTransformation);

	  P4_naive.x = genericTransformation.getOrigin().x();
	  P4_naive.y = genericTransformation.getOrigin().y();
	  P4_naive.z = genericTransformation.getOrigin().z();
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	}	
	
    bool lineIntersectionSuccess = false,lineIntersectionSuccessNaive = false;	
    lineIntersectionSuccess = LineLineIntersect(P1,P2,P3,P4,&Pa,&Pb,&mua,&mub);
    lineIntersectionSuccessNaive = LineLineIntersect(P1,P2,P3,P4_naive,&Pa_naive,&Pb_naive,&muaNaive,&mubNaive);
    //ROS_INFO("PaX = %f",Pa.x);
    //ROS_INFO("PaY = %f",Pa.y);
    //ROS_INFO("PaZ = %f",Pa.z);
    //ROS_INFO("PbX = %f",Pb.x);
    //ROS_INFO("PbY = %f",Pb.y);
    //ROS_INFO("PbZ = %f",Pb.z);    
    
    if(lineIntersectionSuccess)
    {
      geometry_msgs::PoseWithCovarianceStamped tgt_WFrame;
      
      tgt_WFrame.pose.pose.position.x = (Pa.x + Pb.x)/2;
      tgt_WFrame.pose.pose.position.y = (Pa.y + Pb.y)/2;
      tgt_WFrame.pose.pose.position.z = (Pa.z + Pb.z)/2;
      tgt_WFrame.pose.pose.orientation.w = 1;
      tgt_WFrame.pose.pose.orientation.x = 0;
      tgt_WFrame.pose.pose.orientation.y = 0;
      tgt_WFrame.pose.pose.orientation.z = 0; 
      tgt_WFrame.header.frame_id = "/world_link";
      tgt_WFrame.header.stamp = selfMessageTime;
      
      //ROS_INFO("Pa.x - Pb.x = %f",Pa.x - Pb.x);
      //ROS_INFO("Pa.y - Pb.y = %f",Pa.y - Pb.y);
      //ROS_INFO("Pa.z - Pb.z = %f",Pa.z - Pb.z);      
      
      //the detection is spurious if the point p_closest_cam2 lies outside the mate blob
      if(distClosestPointFromBlobCenter<((mateBlob.poses[5].position.x + mateBlob.poses[5].position.y)/2))
       stereoSuccess = true;
      else
       stereoSuccess = false;
      
      if(stereoSuccess)
      {      
	transform1.setOrigin( tf::Vector3(tgt_WFrame.pose.pose.position.x, tgt_WFrame.pose.pose.position.y, tgt_WFrame.pose.pose.position.z));
	      
	br.sendTransform(tf::StampedTransform(transform1, selfMessageTime, "world_link", detectedObjectLink)); 
	
	geometry_msgs::PoseWithCovariance poseObjOpticalframe_,poseObjWorld_;
	geometry_msgs::Pose poseOpticalframeWorld_;
	
  
	poseOpticalframeWorld_.position.x = SelfOptFrameInWorldTransform.getOrigin().x();
	poseOpticalframeWorld_.position.y = SelfOptFrameInWorldTransform.getOrigin().y();
	poseOpticalframeWorld_.position.z = SelfOptFrameInWorldTransform.getOrigin().z();
	
	poseOpticalframeWorld_.orientation.w = SelfOptFrameInWorldTransform.getRotation().getW();
	poseOpticalframeWorld_.orientation.x = SelfOptFrameInWorldTransform.getRotation().getX();
	poseOpticalframeWorld_.orientation.y = SelfOptFrameInWorldTransform.getRotation().getY();
	poseOpticalframeWorld_.orientation.z = SelfOptFrameInWorldTransform.getRotation().getZ();	  
	  

	try
	{
	  //lr.waitForTransform(selfCamRGBOpticalFrameLink, detectedObjectLink, selfMessageTime, ros::Duration(3.0));
	  lr.lookupTransform(selfCamRGBOpticalFrameLink, detectedObjectLink, ros::Time(0), genericTransformation);

	  poseObjOpticalframe_.pose.position.x = genericTransformation.getOrigin().x();
	  poseObjOpticalframe_.pose.position.y = genericTransformation.getOrigin().y();
	  poseObjOpticalframe_.pose.position.z = genericTransformation.getOrigin().z();
	  poseObjOpticalframe_.pose.orientation.w = 1;
	  poseObjOpticalframe_.pose.orientation.x = 0;
	  poseObjOpticalframe_.pose.orientation.y = 0;
	  poseObjOpticalframe_.pose.orientation.z = 0;
	  
	  double devFactor = 0.05*distClosestPointFromBlobCenter;
	  double x_dev = devFactor*poseObjOpticalframe_.pose.position.x;
	  double y_dev = devFactor*poseObjOpticalframe_.pose.position.y;
	  double z_dev = devFactor*poseObjOpticalframe_.pose.position.z;
	  
	  poseObjOpticalframe_.covariance[18+3] = 0.01;
	  poseObjOpticalframe_.covariance[24+4] = 0.01;
	  poseObjOpticalframe_.covariance[30+5] = 0.01;
	  poseObjOpticalframe_.covariance[0] = x_dev*x_dev;
	  poseObjOpticalframe_.covariance[7] = y_dev*y_dev;
	  poseObjOpticalframe_.covariance[14] = z_dev*z_dev;	

	  pose_cov_ops::compose(poseOpticalframeWorld_,poseObjOpticalframe_,  poseObjWorld_);
	  
	  //replace tgt_WFrame with the new thing after covariance transformation!
	  tgt_WFrame.pose.covariance = poseObjWorld_.covariance;	  
	  
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  
	  return;
	}	
	
      
	
	objWorldPub_.publish(tgt_WFrame);

      }  
    }
    
    if(lineIntersectionSuccessNaive)
    {
      geometry_msgs::PoseWithCovarianceStamped tgt_WFrame;
      
      tgt_WFrame.pose.pose.position.x = (Pa_naive.x + Pb_naive.x)/2;
      tgt_WFrame.pose.pose.position.y = (Pa_naive.y + Pb_naive.y)/2;
      tgt_WFrame.pose.pose.position.z = (Pa_naive.z + Pb_naive.z)/2;
      tgt_WFrame.pose.pose.orientation.w = 1;
      tgt_WFrame.pose.pose.orientation.x = 0;
      tgt_WFrame.pose.pose.orientation.y = 0;
      tgt_WFrame.pose.pose.orientation.z = 0; 
      tgt_WFrame.header.frame_id = "/world_link";
      tgt_WFrame.header.stamp = selfMessageTime;
	
      objWorldPubNaive_.publish(tgt_WFrame);
    }
    
       
  }

}

void Stereo_Detector::storeMateCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  mate_camFx = msg->K[0];
  mate_camFy = msg->K[4];
  
  mate_camCx = msg->K[2];
  mate_camCy = msg->K[5];
  
  ROS_INFO("mate cameraFx = %f",mate_camFx);
  ROS_INFO("mate cameraFy = %f",mate_camFy);
  
  //storing these parameters only once is required 
  mateCamInfoSub_.shutdown();
}

void Stereo_Detector::selfBlobCallBack(const geometry_msgs::PoseArray::ConstPtr& msg)
{

  selfBlob.header = msg->header; 
  selfBlob.poses = msg->poses;
  selfMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  performStereoMatching();
}

void Stereo_Detector::mateBlobCallBack(const geometry_msgs::PoseArray::ConstPtr& msg)
{

  mateBlob.header = msg->header; 
  mateBlob.poses = msg->poses;  
  mateMessageTime = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  
}


int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_stereo_detector");

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
  Stereo_Detector node(nh);
    
  spin();
  
  return 0;  
}
#include "../include/detector.h" 


void Detector::storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ros::Time actualMessageTime(msg->header.stamp.sec, msg->header.stamp.nsec);

  tfRobWorld.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tfRobWorld.setRotation(q);
  
  poseRobWorld_.position.x = msg->pose.position.x;
  poseRobWorld_.position.y = msg->pose.position.y;
  poseRobWorld_.position.z = msg->pose.position.z;
  poseRobWorld_.orientation.w = msg->pose.orientation.w;
  poseRobWorld_.orientation.x = msg->pose.orientation.x;
  poseRobWorld_.orientation.y = msg->pose.orientation.y;
  poseRobWorld_.orientation.z = msg->pose.orientation.z;
  
  br.sendTransform(tf::StampedTransform(tfRobWorld, actualMessageTime, "world_link", robotBaseLink));
}

void Detector::storeLatestObjectGTPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ros::Time actualMessageTime(msg->header.stamp.sec, msg->header.stamp.nsec);

  tfRobWorld.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tfRobWorld.setRotation(q);
  
  poseRobWorld_.position.x = msg->pose.position.x;
  poseRobWorld_.position.y = msg->pose.position.y;
  poseRobWorld_.position.z = msg->pose.position.z;
  poseRobWorld_.orientation.w = msg->pose.orientation.w;
  poseRobWorld_.orientation.x = msg->pose.orientation.x;
  poseRobWorld_.orientation.y = msg->pose.orientation.y;
  poseRobWorld_.orientation.z = msg->pose.orientation.z;
  
  br.sendTransform(tf::StampedTransform(tfRobWorld, actualMessageTime, "world_link", "GT_object_link"));
}




void Detector::storeCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camFx = msg->K[0];
  camFy = msg->K[4];
  
  camCx = msg->K[2];
  camCy = msg->K[5];
  
  ROS_INFO("cameraFx = %f",camFx);
  ROS_INFO("cameraFy = %f",camFy);
  
  //storing these parameters only once is required 
  camInfoSub_.shutdown();
}



void Detector::combinedDetection(const sensor_msgs::Image::ConstPtr& image)
{

  ROS_INFO("Received a base Image");
  
  
}


void Detector::featureBasedDetection(const sensor_msgs::Image::ConstPtr& image)
{

  ROS_INFO("Received a base Image");
  
  
}


void Detector::segmentationBasedDetection(const sensor_msgs::Image::ConstPtr& image)
{
  //ROS_INFO("Received a mask Image");
  ros::Time actualMessageTime(image->header.stamp.sec, image->header.stamp.nsec);

  br.sendTransform(tf::StampedTransform(tfCamRob, actualMessageTime, robotBaseLink, camBaseLink));   
  
  cv_bridge::CvImagePtr cv_ptr;
  
  try
  {
      cv_ptr = cv_bridge::toCvCopy(image, "mono8");
  }
  catch (cv_bridge::Exception& e)
  {
      //if there is an error during conversion, display it
      ROS_ERROR("Detector error when reading image on the segmentation mask image topic: %s", e.what());
      return;
  }    
   
  int i = 0, count = 0;
  
  vector<vector<Point> > contour_;
  vector<Vec4i> hierarchy;
  
  findContours ( cv_ptr->image , contour_, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  #ifdef DRAW_DEBUG  
    Mat dst = Mat::zeros(cv_ptr->image.size(),CV_8UC3);
  #endif
      
  int idx = 0, largestContourIndex = 0;
  double areaOfLargestContour = 0; 
  
  // If we found at least one contour
  if(contour_.size()>0)
  {
    // find the biggest contour
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
	const vector<Point>& c = contour_[idx];
	double area = fabs(contourArea(Mat(c)));
	if( area > areaOfLargestContour )
	{
	    areaOfLargestContour = area;
	    largestContourIndex = idx;
	}	
    }
  
    if(areaOfLargestContour > minAreaOfTargetProject)
    {
      // find the convex hull
      vector<vector<Point> >hull(1);
      convexHull( Mat(contour_[largestContourIndex]), hull[0], false );
      
      /// Find the rotated rectangles and ellipses for each contour
      vector<RotatedRect> minRect(1);
      vector<RotatedRect> minEllipse(1);  
      
      minRect[0] = minAreaRect( Mat(contour_[largestContourIndex]) );
	  if( contour_[largestContourIndex].size() > 5 )
	    { minEllipse[i] = fitEllipse(Mat(contour_[largestContourIndex])); }
	    
      Point2f objectCenterIm = minRect[0].center;
      double objectSizeInImage = minRect[0].size.width*minRect[0].size.height;
      	
      
      // Check if the bounding rectangle is totally inside the image 
      Point2f rectangle_points[4]; minRect[0].points(rectangle_points);
      bool isRectFullyInsideImage = true;
	for( int j = 0; j < 4; j++ )
	{
	  if(!(rectangle_points[j].x>2 &&  rectangle_points[j].y>2 &&  rectangle_points[j].x < (cv_ptr->image.cols - 2) &&  rectangle_points[j].y < (cv_ptr->image.rows - 2)))
	   isRectFullyInsideImage = false;	
	}
      
      if(isRectFullyInsideImage)
      {
	Z_obj_camfrm = pow((camFx*camFy*objectRealSurfaceArea/objectSizeInImage),0.5);
	X_obj_camfrm = Z_obj_camfrm*(objectCenterIm.x-camCx)/camFx;
	Y_obj_camfrm = Z_obj_camfrm*(objectCenterIm.y-camCy)/camFy;
	
	double x_mean=meanFactor*X_obj_camfrm;
	double y_mean=meanFactor*Y_obj_camfrm;
	double z_mean=meanFactor*Z_obj_camfrm;
	
	double x_dev = 4*devFactor*X_obj_camfrm;
	double y_dev = 4*devFactor*Y_obj_camfrm;
	double z_dev = devFactor*Z_obj_camfrm;
    
	tfObCam.setOrigin( tf::Vector3(x_mean, y_mean, z_mean));
	tf::Quaternion q(0,0,0,1);
	tfObCam.setRotation(q);
	
	br.sendTransform(tf::StampedTransform(tfObCam, actualMessageTime, camRGBOpticalFrameLink, detectedObjectLink));   
	
	//Now get the target in the world link frame
	try
	{
	  listenerObjWorld.lookupTransform("/world_link", detectedObjectLink, ros::Time(0), transformObjWorld);
	  
	  poseObjWorld.header.frame_id = "/world_link";
	  //poseObjWorld.header.stamp = rospy.Time.now();
	  
	  poseObjWorld.pose.pose.position.x = transformObjWorld.getOrigin().x();
	  poseObjWorld.pose.pose.position.y = transformObjWorld.getOrigin().y();
	  poseObjWorld.pose.pose.position.z = transformObjWorld.getOrigin().z();
	  
	  poseObjWorld.pose.pose.orientation.w = transformObjWorld.getRotation().getW();
	  poseObjWorld.pose.pose.orientation.x = transformObjWorld.getRotation().getX();
	  poseObjWorld.pose.pose.orientation.y = transformObjWorld.getRotation().getY();
	  poseObjWorld.pose.pose.orientation.z = transformObjWorld.getRotation().getX();
	  
	  objWorldPub_.publish(poseObjWorld);	
	}      
	
	catch (tf::TransformException &ex) 
	{
	  ROS_WARN("%s",ex.what());
	  ros::Duration(1.0).sleep();
	  return;
	}      
	
	///@hack for February demo. @Fix this
	// We project the detected objected pose to the ground plane in the direction of the camera
	{
	  try
	  {
	    listenerCamWorld.lookupTransform("/world_link", camRGBOpticalFrameLink, ros::Time(0), transformCamWorld);
	    
	    objWorldFrame[0] = poseObjWorld.pose.pose.position.x;
	    objWorldFrame[1] = poseObjWorld.pose.pose.position.y;
	    objWorldFrame[2] = poseObjWorld.pose.pose.position.z;	  
	    
	    camOptCenterWorld[0] = transformCamWorld.getOrigin().x();
	    camOptCenterWorld[1] = transformCamWorld.getOrigin().y();
	    camOptCenterWorld[2] = transformCamWorld.getOrigin().z();
	    
	    double a = camOptCenterWorld[0]-objWorldFrame[0]; double b = camOptCenterWorld[1]-objWorldFrame[1]; double c = camOptCenterWorld[2]-objWorldFrame[2]; double t=0;
	    if(c!=0)
	      t = (0-objWorldFrame[2])/c; // intersecting the line with the plane z=0
	      
	    projectedPoseObjWorld.pose.pose.position.x = objWorldFrame[0] + a*t;
	    projectedPoseObjWorld.pose.pose.position.y = objWorldFrame[1] + b*t;
	    projectedPoseObjWorld.pose.pose.position.z = 0.0;
	    
	    //projectedObjWorldPub_.publish(projectedPoseObjWorld);
	    
	    // Also create a transformation now
	    tfProjectedObWorld.setOrigin( tf::Vector3(projectedPoseObjWorld.pose.pose.position.x, projectedPoseObjWorld.pose.pose.position.y, 0.0));
	    tf::Quaternion q1(0,0,0,1);
	    tfProjectedObWorld.setRotation(q1);
	    
	    br.sendTransform(tf::StampedTransform(tfProjectedObWorld, actualMessageTime, "world_link", projectedObjectLink));	  	    
	  }     
	  
	  catch (tf::TransformException &ex) 
	  {
	    ROS_WARN("%s",ex.what());
	    ros::Duration(1.0).sleep();
	    return;
	  }   
	}
	{
	  try
	  {
	    listenerprojObjCam.lookupTransform(camRGBOpticalFrameLink,projectedObjectLink, ros::Time(0), transformprojObjCam);
	    
	    
	    
	    poseOpticalframeWorld_.position.x = camOptCenterWorld[0];
	    poseOpticalframeWorld_.position.y = camOptCenterWorld[1];
	    poseOpticalframeWorld_.position.z = camOptCenterWorld[2];
	    
	    poseOpticalframeWorld_.orientation.w = transformCamWorld.getRotation().getW();
	    poseOpticalframeWorld_.orientation.x = transformCamWorld.getRotation().getX();
	    poseOpticalframeWorld_.orientation.y = transformCamWorld.getRotation().getY();
	    poseOpticalframeWorld_.orientation.z = transformCamWorld.getRotation().getZ();
	    
	    ///@hack this is now the ground-projected pose of object in the camera frame 
	    poseObjOpticalframe_.pose.position.x = transformprojObjCam.getOrigin().x();
	    poseObjOpticalframe_.pose.position.y = transformprojObjCam.getOrigin().y();
	    poseObjOpticalframe_.pose.position.z = transformprojObjCam.getOrigin().z();
	    
	    poseObjOpticalframe_.pose.orientation.w = 1;
	    poseObjOpticalframe_.pose.orientation.x = 0;
	    poseObjOpticalframe_.pose.orientation.y = 0;
	    poseObjOpticalframe_.pose.orientation.z = 0;
	    
	    poseObjOpticalframe_.covariance[18+3] = 0.01;
	    poseObjOpticalframe_.covariance[24+4] = 0.01;
	    poseObjOpticalframe_.covariance[30+5] = 0.01;
	    poseObjOpticalframe_.covariance[0] = x_dev*x_dev;
	    poseObjOpticalframe_.covariance[7] = y_dev*y_dev;
	    poseObjOpticalframe_.covariance[14] = z_dev*z_dev;	    
	    
	    
	    pose_cov_ops::compose(poseOpticalframeWorld_,poseObjOpticalframe_,  poseObjWorld_);

	    poseObjWorldStamped_.pose = poseObjWorld_;
	    poseObjWorldStamped_.header.frame_id = "/world_link";
	    poseObjWorldStamped_.header.stamp = actualMessageTime;
	    projectedObjWorldPub_.publish(poseObjWorldStamped_);
	    
	  }     
	  
	  catch (tf::TransformException &ex) 
	  {
	    ROS_WARN("%s",ex.what());
	    ros::Duration(1.0).sleep();
	    return;
	  }   
	}	
	
      #ifdef TERMINAL_DEBUG
      
	ROS_INFO("objectCenterIm.x in pixels = %d",objectCenterIm.x);
	ROS_INFO("objectCenterIm.y in pixels = %d",objectCenterIm.y);
	
	ROS_INFO("camFx in meter = %f",camFx);
	ROS_INFO("objectRealSurfaceArea in meter square = %f",objectRealSurfaceArea);
	ROS_INFO("objectSizeInImage in pixels = %f",objectSizeInImage);    
	
	ROS_INFO("X_obj_camfrm in meter = %f",X_obj_camfrm);
	ROS_INFO("Y_obj_camfrm in meter = %f",Y_obj_camfrm);
	ROS_INFO("Z_obj_camfrm in meter = %f",Z_obj_camfrm);  

      #endif
	
	    
      #ifdef DRAW_DEBUG
	  Scalar color( rand()&255, rand()&255, rand()&255 );
	  //drawContours( dst, contour_, largestContourIndex, color, 1, 8, hierarchy );
	  
	  Scalar color1( rand()&255, rand()&255, rand()&255 );
	  //drawContours( dst, hull, i, color1, 1, 8, vector<Vec4i>(), 0, Point() );
	  
	  Scalar color2( rand()&255, rand()&255, rand()&255 );
	  //ellipse( dst, minEllipse[i], color2, 2, 8 );
	  // rotated rectangle
	  Scalar color3( rand()&255, rand()&255, rand()&255 );
	  Point2f rect_points[4]; minRect[0].points(rect_points);
	  for( int j = 0; j < 4; j++ )
	    line( dst, rect_points[j], rect_points[(j+1)%4], color3, 1, 8 );
	  namedWindow( "Components", 1 );
	  imshow( "Components", dst );
	  waitKey(2);	
      #endif	
	
      }
      
    }
  }
    
  
}

void Detector::segmentationBasedDetectionUnknownSize(const sensor_msgs::Image::ConstPtr& image)
{
  //ROS_INFO("Received a mask Image");
  ros::Time actualMessageTime(image->header.stamp.sec, image->header.stamp.nsec);

  br.sendTransform(tf::StampedTransform(tfCamRob, actualMessageTime, robotBaseLink, camBaseLink));   
  
  cv_bridge::CvImagePtr cv_ptr;
  
  try
  {
      cv_ptr = cv_bridge::toCvCopy(image, "mono8");
  }
  catch (cv_bridge::Exception& e)
  {
      //if there is an error during conversion, display it
      ROS_ERROR("Detector error when reading image on the segmentation mask image topic: %s", e.what());
      return;
  }    
   
  int i = 0, count = 0;
  
  vector<vector<Point> > contour_;
  vector<Vec4i> hierarchy;
  
  findContours ( cv_ptr->image , contour_, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  #ifdef DRAW_DEBUG  
    Mat dst = Mat::zeros(cv_ptr->image.size(),CV_8UC3);
  #endif
      
  int idx = 0, largestContourIndex = 0;
  double areaOfLargestContour = 0; 
  
  // If we found at least one contour
  if(contour_.size()>0)
  {
    // find the biggest contour
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
	const vector<Point>& c = contour_[idx];
	double area = fabs(contourArea(Mat(c)));
	if( area > areaOfLargestContour )
	{
	    areaOfLargestContour = area;
	    largestContourIndex = idx;
	}	
    }
  
    if(areaOfLargestContour > minAreaOfTargetProject)
    {
      // find the convex hull
      vector<vector<Point> >hull(1);
      convexHull( Mat(contour_[largestContourIndex]), hull[0], false );
      
      /// Find the rotated rectangles and ellipses for each contour
      vector<RotatedRect> minRect(1);
      vector<RotatedRect> minEllipse(1);  
      
      minRect[0] = minAreaRect( Mat(contour_[largestContourIndex]) );
	  if( contour_[largestContourIndex].size() > 5 )
	    { minEllipse[i] = fitEllipse(Mat(contour_[largestContourIndex])); }
	    
      Point2f objectCenterIm = minRect[0].center;
      double objectSizeInImage = minRect[0].size.width*minRect[0].size.height;
      	
      
      // Check if the bounding rectangle is totally inside the image 
      Point2f rectangle_points[4]; minRect[0].points(rectangle_points);
      bool isRectFullyInsideImage = true;
	for( int j = 0; j < 4; j++ )
	{
	  if(!(rectangle_points[j].x>2 &&  rectangle_points[j].y>2 &&  rectangle_points[j].x < (cv_ptr->image.cols - 2) &&  rectangle_points[j].y < (cv_ptr->image.rows - 2)))
	   isRectFullyInsideImage = false;	
	}
      
      if(isRectFullyInsideImage)
      {
	geometry_msgs::PoseArray blobInImage;
	
	blobInImage.header = image->header;
	blobInImage.header.frame_id = camRGBOpticalFrameLink;
	
	geometry_msgs::Pose tempPose;
	tempPose.position.x = objectCenterIm.x;
	tempPose.position.y = objectCenterIm.y;
	blobInImage.poses.push_back(tempPose);
	
	for( int j = 0; j < 4; j++ )
	{
	  tempPose.position.x = rectangle_points[j].x;
	  tempPose.position.y = rectangle_points[j].y;	
	  blobInImage.poses.push_back(tempPose);
	}	

	tempPose.position.x = minRect[0].size.width;
	tempPose.position.y = minRect[0].size.height;	
	blobInImage.poses.push_back(tempPose);	
	
	blobPublisher_.publish(blobInImage);

      #ifdef TERMINAL_DEBUG
      
	ROS_INFO("objectCenterIm.x in pixels = %f",objectCenterIm.x);
	ROS_INFO("objectCenterIm.y in pixels = %f",objectCenterIm.y);
	
	ROS_INFO("camFx in meter = %f",camFx);
	ROS_INFO("objectRealSurfaceArea in meter square = %f",objectRealSurfaceArea);
	ROS_INFO("objectSizeInImage in pixels = %f",objectSizeInImage);    
	
	ROS_INFO("X_obj_camfrm in meter = %f",X_obj_camfrm);
	ROS_INFO("Y_obj_camfrm in meter = %f",Y_obj_camfrm);
	ROS_INFO("Z_obj_camfrm in meter = %f",Z_obj_camfrm);  

      #endif
	
	    
      #ifdef DRAW_DEBUG
	  Scalar color( rand()&255, rand()&255, rand()&255 );
	  //drawContours( dst, contour_, largestContourIndex, color, 1, 8, hierarchy );
	  
	  Scalar color1( rand()&255, rand()&255, rand()&255 );
	  //drawContours( dst, hull, i, color1, 1, 8, vector<Vec4i>(), 0, Point() );
	  
	  Scalar color2( rand()&255, rand()&255, rand()&255 );
	  //ellipse( dst, minEllipse[i], color2, 2, 8 );
	  // rotated rectangle
	  Scalar color3( rand()&255, rand()&255, rand()&255 );
	  Point2f rect_points[4]; minRect[0].points(rect_points);
	  for( int j = 0; j < 4; j++ )
	    line( dst, rect_points[j], rect_points[(j+1)%4], color3, 1, 8 );
	  namedWindow( "Components", 1 );
	  imshow( "Components", dst );
	  waitKey(2);	
      #endif	
	
      }
      
    }
  }
    
  
}



int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_detector");

  if (argc < 3)
    {
      ROS_WARN("WARNING: you should specify i) the detector type as 'HISTOGRAM' or 'FEATURE' or 'COMBINED' and ii) whether or known the object size is known or not as 'KNOWN_SIZE' and 'UNKNOWN_SIZE' \n");
      return 1;
    }
  else
  {
    ROS_INFO("Detector provided = %s for %s",argv[1],argv[2]);
  }
      
  ros::NodeHandle nh("~");  
  Detector node(nh,argv[1],argv[2]);
    
  spin();
  
  return 0;  
}
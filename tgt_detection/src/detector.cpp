#include "../include/detector.h" 


void Detector::storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ros::Time actualMessageTime(msg->header.stamp.sec, msg->header.stamp.nsec);

  tfRobWorld.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tfRobWorld.setRotation(q);
  
  br.sendTransform(tf::StampedTransform(tfRobWorld, actualMessageTime, "world_link", robotBaseLink));
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
	
	br.sendTransform(tf::StampedTransform(tfObCam, actualMessageTime, camRGBOpticalFrameLink, objectLink));
	br.sendTransform(tf::StampedTransform(tfCamRob, actualMessageTime, robotBaseLink, camBaseLink));      
	
	//Now get the target in the world link frame
	try
	{
	  listenerObjWorld.lookupTransform("/world_link", objectLink, ros::Time(0), transformObjWorld);
	  
	  poseObjWorld.header.frame_id = "/world_link";
	  //poseObjWorld.header.stamp = rospy.Time.now();
	  
	  poseObjWorld.pose.pose.position.x = transformObjWorld.getOrigin().x();
	  poseObjWorld.pose.pose.position.y = transformObjWorld.getOrigin().y();
	  poseObjWorld.pose.pose.position.z = transformObjWorld.getOrigin().z();
	  
	  poseObjWorld.pose.pose.orientation.w = 1.0;
	  poseObjWorld.pose.pose.orientation.x = 0.0;
	  poseObjWorld.pose.pose.orientation.y = 0.0;
	  poseObjWorld.pose.pose.orientation.z = 0.0;

	  poseObjWorld.pose.covariance[18+3] = 0.01;
	  poseObjWorld.pose.covariance[24+4] = 0.01;
	  poseObjWorld.pose.covariance[30+5] = 0.01;
	  poseObjWorld.pose.covariance[0] = x_dev*x_dev;
	  poseObjWorld.pose.covariance[7] = y_dev*y_dev;
	  poseObjWorld.pose.covariance[14] = z_dev*z_dev;
	  
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
	    
	    double x1,y1,z1,x2,y2,z2;
	    
	    x1 = poseObjWorld.pose.pose.position.x;
	    y1 = poseObjWorld.pose.pose.position.y;
	    z1 = poseObjWorld.pose.pose.position.z;	  
	    
	    x2 = transformCamWorld.getOrigin().x();
	    y2 = transformCamWorld.getOrigin().y();
	    z2 = transformCamWorld.getOrigin().z();
	    
	    double a = x2-x1; double b = y2-y1; double c = z2-z1; double t=0;
	    if(c!=0)
	      t = (0-z1)/c; // intersecting the line with the plane z=0
	      
	    projectedPoseObjWorld.pose.pose.position.x = x1 + a*t;
	    projectedPoseObjWorld.pose.pose.position.y = y1 + b*t;
	    projectedPoseObjWorld.pose.pose.position.z = 0.0;
	    
	    projectedObjWorldPub_.publish(projectedPoseObjWorld);
	    
	    // Also create a transformation now
	    tfProjectedObWorld.setOrigin( tf::Vector3(projectedPoseObjWorld.pose.pose.position.x, projectedPoseObjWorld.pose.pose.position.y, 0.0));
	    tf::Quaternion q1(0,0,0,1);
	    tfProjectedObWorld.setRotation(q1);
	    
	    br.sendTransform(tf::StampedTransform(tfProjectedObWorld, actualMessageTime, "world_link", "projected_objectLink"));	  
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

int main(int argc, char* argv[])
{
  
  ros::init(argc, argv, "tgt_detector");

  if (argc < 2)
    {
      ROS_WARN("WARNING: you should at least specify the detector type as 'HISTOGRAM' or 'FEATURE' or 'COMBINED' of \n");
      return 1;
    }
  else
  {
    ROS_INFO("Detector provided = %s",argv[1]);
  }
      
  ros::NodeHandle nh("~");  
  Detector node(nh,argv[1]);
    
  spin();
  
  return 0;  
}
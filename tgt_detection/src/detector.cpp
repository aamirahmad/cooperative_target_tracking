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
      
      Z_obj_camfrm = pow((camFx*camFy*objectRealSurfaceArea/objectSizeInImage),0.5);
      X_obj_camfrm = Z_obj_camfrm*(objectCenterIm.x-camCx)/camFx;
      Y_obj_camfrm = Z_obj_camfrm*(objectCenterIm.y-camCy)/camFy;
      
  
      tfObCam.setOrigin( tf::Vector3(X_obj_camfrm, Y_obj_camfrm, Z_obj_camfrm));
      tf::Quaternion q(0,0,0,1);
      tfObCam.setRotation(q);
      
      br.sendTransform(tf::StampedTransform(tfObCam, actualMessageTime, camRGBOpticalFrameLink, objectLink));
      br.sendTransform(tf::StampedTransform(tfCamRob, actualMessageTime, robotBaseLink, camBaseLink));      
      
//       std::cout<<"objectCenterIm.x in pixels = "<<objectCenterIm.x<<std::endl;
//       std::cout<<"objectCenterIm.y in pixels = "<<objectCenterIm.y<<std::endl;
      
//       std::cout<<"camFx in meter = "<<camFx<<std::endl;
//       std::cout<<"objectRealSurfaceArea in meter square= "<<objectRealSurfaceArea<<std::endl;
//       std::cout<<"objectSizeInImage in pixels = "<<objectSizeInImage<<std::endl;    
// 
//       
//       std::cout<<"X_obj_camfrm in meter = "<<X_obj_camfrm<<std::endl;
//       std::cout<<"Y_obj_camfrm in meter = "<<Y_obj_camfrm<<std::endl;
//       std::cout<<"Z_obj_camfrm in meter = "<<Z_obj_camfrm<<std::endl;      

      
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
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/time.h> 

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/io/vpVideoReader.h>
#include <visp_ros/vpROSGrabber.h>

using namespace std;
using namespace ros;

class Detector
{
    bool detectionSucces_; // /*!< true if detector detects the object in the image */
    
    Subscriber imageSub_;
    string imageTopic;
  
  public:
    Detector(NodeHandle *nh_)
    {
      nh_->getParam("IMAGE_TOPIC", imageTopic);
      
      imageSub_ = nh_->subscribe<sensor_msgs::Image>(imageTopic, 1000, boost::bind(&Detector::imageCallback,this, _1));
    }
    
    /*! \brief This is the image callback. Most of the detection processing should happen here.
      * However, each sub-step of detection, e.g., image segmentation, feature detection, etc.,  
      * should happen in the respective functions which can be called from here
      *  Detailed description starts here.
      */
    void imageCallback(const sensor_msgs::Image::ConstPtr&);

};
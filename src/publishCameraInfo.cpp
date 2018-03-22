//contains cameraInfo for snapcam.
//MUST RENAME NODE:  snap_cam_highres_publisher/image (etc) to image_raw
//MUST SET CAMFILE:  rosparam set camfile snapcam1 (etc)
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

/**Code adapted from
http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
  More efficient code to modify image before republishing at
http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
*/
class publishCameraInfo
{
public:
  publishCameraInfo(const ros::NodeHandle &nh, const ros::NodeHandle n): nh_(nh), n_(n)
  {
      if(n_.getParam("topic/left", left_topic))
          ROS_INFO("Get left image info topic: %s", left_topic.c_str());
      else
          ROS_WARN("Use default left image info topic: %s", left_topic.c_str());

      if(n_.getParam("topic/right", right_topic))
          ROS_INFO("Get right image info topic: %s", right_topic.c_str());
      else
          ROS_WARN("Use default right image info topic: %s", right_topic.c_str())

      pub_left_cam_info = nh_.advertise<sensor_msgs::CameraInfo>(left_topic, 1);
      pub_right_cam_info = nh_.advertise<sensor_msgs::CameraInfo>(right_topic, 1);

//      sub_left = nh_.subscribe("image_raw", 1, &publishCameraInfo::callback, this);
  }
  void callback(const sensor_msgs::Image& imgmsg1, const sensor_msgs::Image& imgmsg2)
  {
    //not matching camname to camera name in input file throws an error in camera_info_manager.
    //This error can be ignored.
//    const std::string camurl="file:///home/rnl/.ros/camera_info/snapcam1.yaml";
    std::string camname;
    n_.getParam ( "camfile", camname);  //camname is set by:  rosparam set camfile _____
    const std::string camnameConst=camname;
    const std::string camurlRead="file:///home/rnl/.ros/camera_info/" + camname + ".yaml";
    camera_info_manager::CameraInfoManager caminfo(n_, camnameConst,camurlRead);
//    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(caminfo->getCameraInfo()));  //copied
//    camera_info_manager::CameraInfoManager caminfo(n_, camname);
    sensor_msgs::CameraInfo ci;
    ci=caminfo.getCameraInfo();

    ci.header.stamp = imgmsg1.header.stamp;
    ci.header.frame_id = imgmsg1.header.frame_id;
    
    // Publish via image_transport
      pub_left_cam_info.publish(ci);
  }

  private:
  ros::NodeHandle n_,nh_;
  ros::Publisher pub_left_cam_info, pub_right_cam_info;
  ros::Subscriber sub_left, sub_right;

  string left_topic = "/wide/left/camera_info";
  string right_topic = "/wide/right/camera_info";

};//End of class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publishCameraInfo");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    publishCameraInfo cameraPubObject(nh, n);

    message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, "/wide/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, "/wide/right/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&publishCameraInfo::callback, &cameraPubObject ,_1, _2));

    ros::spin();

  return 0;
}




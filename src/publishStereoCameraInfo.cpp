//contains cameraInfo for stereo cameras with rec and proj matrix.

#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

/**Code adapted from
https://github.com/nickMont/cameraInfoPublisher
*/
class publishCameraInfo
{
public:
  publishCameraInfo(const ros::NodeHandle &nh, const ros::NodeHandle &n): nh_(nh), n_(n)
  {
      n_.getParam ( "camfile_left", camname_left);  //camname is set by:  rosparam set camfile _____

      n_.getParam ( "camfile_right", camname_right);

      pub_left_cam_info = nh_.advertise<sensor_msgs::CameraInfo>(camname_left + "/camera_info", 1);

      pub_right_cam_info = nh_.advertise<sensor_msgs::CameraInfo>(camname_right + "/camera_info", 1);

  }
  void callback(const sensor_msgs::ImageConstPtr& imgmsg1, const sensor_msgs::ImageConstPtr& imgmsg2)
  {
      const std::string camnameConst_l = camname_left;

      const std::string camnameConst_r = camname_right;

      const std::string camurlRead_l = "file://" + ros::package::getPath("undistort_images") + "/calib_files/" + camname_left + ".yaml";

      const std::string camurlRead_r = "file://" + ros::package::getPath("undistort_images") + "/calib_files/" + camname_right + ".yaml";

      camera_info_manager::CameraInfoManager caminfo_l(nh_, camnameConst_l, camurlRead_l);

      camera_info_manager::CameraInfoManager caminfo_r(nh_, camnameConst_r, camurlRead_r);

      ci_left = caminfo_l.getCameraInfo();

      ci_right = caminfo_r.getCameraInfo();

      ci_left.header.stamp = imgmsg1->header.stamp;
      ci_left.header.frame_id = imgmsg1->header.frame_id;

      ci_right.header.stamp = imgmsg2->header.stamp;
      ci_right.header.frame_id = imgmsg2->header.frame_id;

      // Publish via image_transport
      pub_left_cam_info.publish(ci_left);
      pub_right_cam_info.publish(ci_right);
  }

  private:
  ros::NodeHandle n_,nh_;

  ros::Publisher pub_left_cam_info, pub_right_cam_info;

  string left_info_topic, right_info_topic, camname_left, camname_right;

  sensor_msgs::CameraInfo ci_left, ci_right;
};//End of class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publishStereoCameraInfo");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    string left_img_topic = "/wide/left/camera_raw";
    string right_img_topic = "/wide/right/camera_raw";

    if(n.getParam("topic/left_img", left_img_topic))
        ROS_INFO("Get left image info topic: %s", left_img_topic.c_str());
    else
        ROS_WARN("Use default left image info topic: %s", left_img_topic.c_str());

    if(n.getParam("topic/right_img", right_img_topic))
        ROS_INFO("Get right image info topic: %s", right_img_topic.c_str());
    else
        ROS_WARN("Use default right image info topic: %s", right_img_topic.c_str());

    publishCameraInfo cameraPubStereoObject(nh, n);

    message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, left_img_topic, 5);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, right_img_topic, 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&publishCameraInfo::callback, &cameraPubStereoObject ,_1, _2));

    ros::spin();

    return 0;
}




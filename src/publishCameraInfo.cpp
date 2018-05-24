//contains cameraInfo for mono camera.

#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"

using namespace std;

/**Code adapted from
http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
  More efficient code to modify image before republishing at
http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
*/
class publishCameraInfo
{
public:
  publishCameraInfo(const ros::NodeHandle &nh, const ros::NodeHandle &n): nh_(nh), n_(n)
  {
      string img_topic = "/camera_raw";

      n_.getParam ( "camfile", camname);  //camname is set by:  rosparam set camfile _____

      if(n_.getParam("topic/img", img_topic))
          ROS_INFO("Get image info topic: %s", img_topic.c_str());
      else
          ROS_WARN("Use default image info topic: %s", img_topic.c_str());
      if(n_.getParam("topic/info", info_topic))
          ROS_INFO("Get image info topic: %s", info_topic.c_str());
      else
          ROS_WARN("Use default image info topic: %s", info_topic.c_str());

      if(n.getParam("cut", cut_))
          ROS_INFO("Get cut flag: %s", cut_? "true":"false");
      else
          ROS_WARN("Using default cut flag: false!");

      if(n.getParam("x_offset", x_off))
          ROS_INFO("Get x offset: %u", x_off);

      if(n.getParam("y_offset", y_off))
          ROS_INFO("Get x offset: %u", y_off);

      if(n.getParam("width", w_off))
          ROS_INFO("Get width offset: %u", w_off);

      if(n.getParam("height", h_off))
          ROS_INFO("Get height offset: %u", h_off);


      pub_cam_info = nh_.advertise<sensor_msgs::CameraInfo>(camname + info_topic, 1);

      if(cut_)
          pub_cam_info_cut = nh_.advertise<sensor_msgs::CameraInfo>(camname + "_crop" + info_topic, 1);

      sub_img = nh_.subscribe(img_topic, 1, &publishCameraInfo::callback, this);
  }
  void callback(const sensor_msgs::ImageConstPtr& imgmsg)
  {

    const std::string camnameConst = camname;

    const std::string camurlRead = "file://" + ros::package::getPath("undistort_images") + "/calib_files/" + camname + ".yaml";

    camera_info_manager::CameraInfoManager caminfo(nh_, camnameConst, camurlRead);

    sensor_msgs::CameraInfo ci, ci_cut;

    ci = caminfo.getCameraInfo();

    ci.header.stamp = imgmsg->header.stamp;
    ci.header.frame_id = imgmsg->header.frame_id;

    ci_cut = ci;

    ci_cut.binning_x = ci_cut.binning_y = 1;
    ci_cut.roi.height = h_off;
    ci_cut.roi.width = w_off;
    ci_cut.roi.x_offset = x_off;
    ci_cut.roi.y_offset = y_off;
    ci_cut.roi.do_rectify = cut_;

    // Publish via image_transport
    pub_cam_info.publish(ci);
    if(cut_)
        pub_cam_info_cut.publish(ci_cut);
  }

  private:
  ros::NodeHandle n_,nh_;
  ros::Publisher pub_cam_info;
  ros::Publisher pub_cam_info_cut;
  ros::Subscriber sub_img;

  bool cut_;

  int x_off, y_off, w_off, h_off;

  string info_topic, camname;

};//End of class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publishCameraInfo");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    publishCameraInfo cameraPubObject(nh, n);

    ros::spin();

    return 0;
}




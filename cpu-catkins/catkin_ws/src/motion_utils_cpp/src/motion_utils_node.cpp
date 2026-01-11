#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "motion_utils_cpp/first_order_filter.h"
#include "motion_utils_cpp/twist_limiter.h"

namespace motion_utils_cpp
{

class MotionUtilsNode
{
public:
  MotionUtilsNode()
    : nh_()
    , pnh_("~")
    , filter_x_(0.3, 0.02, 0.0)
    , filter_y_(0.3, 0.02, 0.0)
    , filter_wz_(0.3, 0.02, 0.0)
    , last_time_(ros::Time::now())
  {
    std::string input_topic;
    std::string output_topic;

    pnh_.param<std::string>("input_cmd_vel_topic", input_topic, std::string("cmd_vel"));
    pnh_.param<std::string>("output_cmd_vel_topic", output_topic, std::string("cmd_vel/smooth_custom"));

    double max_vx, max_vy, max_wz;
    double max_ax, max_ay, max_awz;
    pnh_.param("max_vx", max_vx, 0.5);
    pnh_.param("max_vy", max_vy, 0.5);
    pnh_.param("max_wz", max_wz, 1.0);
    pnh_.param("max_ax", max_ax, 0.5);
    pnh_.param("max_ay", max_ay, 0.5);
    pnh_.param("max_awz", max_awz, 1.0);

    limiter_.setMaxVelocity(max_vx, max_vy, max_wz);
    limiter_.setMaxAcceleration(max_ax, max_ay, max_awz);

    sub_ = nh_.subscribe(input_topic, 1, &MotionUtilsNode::cmdCallback, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic, 1);

    ROS_INFO_STREAM("motion_utils_node listening on '" << input_topic
                    << "', publishing smoothed commands on '" << output_topic << "'");
  }

  void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    const ros::Time now = ros::Time::now();
    const double dt = (now - last_time_).toSec();
    last_time_ = now;

    Twist2D desired;
    desired.vx = filter_x_.update(msg->linear.x);
    desired.vy = filter_y_.update(msg->linear.y);
    desired.wz = filter_wz_.update(msg->angular.z);

    Twist2D limited = limiter_.limit(desired, last_output_, dt);
    last_output_ = limited;

    geometry_msgs::Twist out;
    out.linear.x = limited.vx;
    out.linear.y = limited.vy;
    out.angular.z = limited.wz;
    pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

  FirstOrderFilter filter_x_;
  FirstOrderFilter filter_y_;
  FirstOrderFilter filter_wz_;
  TwistLimiter limiter_;

  Twist2D last_output_;
  ros::Time last_time_;
};

}  // namespace motion_utils_cpp

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_utils_node");
  motion_utils_cpp::MotionUtilsNode node;
  ros::spin();
  return 0;
}



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

std::string tfpre;
std::string target_tf_name;

void poseCallback(const tf2_msgs::TFMessage& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.738141, 1.07112, 2.275) );
  tf::Quaternion q;
  q.setRPY(0, 1.57, 1.57);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", target_tf_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "desk_tf_broadcaster");
  tfpre = argv[1];
  target_tf_name = tfpre+"/deskCam_base_link";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/tf", 10, &poseCallback);

  ros::spin();
  return 0;
};

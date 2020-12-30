#include <ros/ros.h>
#include <server/Correction.h>
#include <std_msgs/Float32.h>

float delta_pos1,delta_pos2;
int checkVal = 0;

std_msgs::Float32 val;

void callback(const server::Correction& msg)
{
  delta_pos1 = msg.delta1;
  delta_pos2 = msg.delta2;

  checkVal = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("correction_val", 1, callback);
  ros::Publisher pub = nh.advertise<std_msgs::Float32>("deviation",1000);
  ros::Rate loop_rate(10);
  ROS_INFO("Image listener initiated");

  while(ros::ok()){

    if(checkVal == 1){
      val.data = delta_pos2;
      pub.publish(val);
      ROS_INFO("Published value");
      checkVal = 0;
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

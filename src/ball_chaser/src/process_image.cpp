#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if(!client.call(srv))
  {
    ROS_ERROR("Failed to call service");
  }
  
}

void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  const int TURN = 1;
  const int MOVE = 5;
  const int STOP = 0;

  int white_count = 0;

  for(int i = 0; i < img.step * img.height; i++)
  {
    if(img.data[i] == white_pixel)
    {
      if(i % img.step < img.step / 3)
      {
        drive_robot(STOP, TURN);
        return;
      }
      else if(i % img.step > img.step * 2 / 3)
      {
        drive_robot(STOP, -(TURN));
        return;
      }
      else
      {
        drive_robot(MOVE, STOP);
        return;
      }
      white_count++;
    }
  }
  if(white_count == 0)
  {
    drive_robot(STOP, STOP);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  ros::spin();

  return 0;
}

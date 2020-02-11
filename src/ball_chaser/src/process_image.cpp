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
  const int SCAN_START = img.data.size() / 3;
  const int SCAN_END = img.data.size() * 2 / 3;

  const int WHITE_PIXEL = 255;
  const float TURN = 1.5;
  const float MOVE = 0.5;
  const float STOP = 0;

  int whiteCount = 0;
  int xSum = 0;

  for(int i = 0; i + 2 < img.data.size(); i+=3)
  {
    int redChannel = img.data[i];
    int blueChannel = img.data[i+1];
    int greenChannel = img.data[i+2];

    if(redChannel == WHITE_PIXEL && blueChannel == WHITE_PIXEL && greenChannel == WHITE_PIXEL)
    {
      whiteCount++;
      xSum += (i % (img.width * 3)) / 3;
    }


  }

  if(whiteCount == 0)
  {
    drive_robot(STOP, STOP);
  }
  else
  {
    int xMean = xSum / whiteCount;
    if(xMean < img.width / 3)
    {
      drive_robot(MOVE, TURN);
      return;
    }
    else if(xMean > img.width * 2 / 3)
    {
      drive_robot(MOVE, -(TURN));
      return;
    }
    else
    {
      drive_robot(MOVE, STOP);
      return;
    }
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

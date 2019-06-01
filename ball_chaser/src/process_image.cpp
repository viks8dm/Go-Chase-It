#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // DONE: Request a service and pass the velocities to it to drive the robot
    //ROS_INFO_STREAM("Driving Robot to new position");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    //ROS_INFO("height= %d, width = %d, step = %d", img.height, img.width, img.step);

    int white_pixel = 255;
    int scan_area_top = img.data.size() / 4;
    int scan_area_bottom = 3 * scan_area_top;
    int num_ball_pixels = 0;
    int ball_pixel_column = 0;
    int section_width = img.width / 3;
    float lin_speed = 0.5, turn_speed = 0.5, stop = 0.0;
    
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = scan_area_top; i<scan_area_bottom+2; i+=3) {
      if (img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel) {
        ball_pixel_column += (i % (3 * img.width)) / 3;
        num_ball_pixels++;
      }
    }

    // check if ball was found
    if (num_ball_pixels==0) {
      drive_robot(stop, stop);
    }
    else {
      int horz_pos = ball_pixel_column / num_ball_pixels;
      if (horz_pos < section_width) {
        drive_robot(lin_speed, turn_speed); // go left
      }
      else if (horz_pos > 2* section_width) {
        drive_robot(lin_speed, -turn_speed); // go right
      }
      else {
        drive_robot(lin_speed, stop); // go forward
      }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

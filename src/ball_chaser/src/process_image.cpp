#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Commanding Robot");

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
    int white_pixel = 255;
    int segment_size = img.step / 3; // divide the image into 3 segments: left, middle and right
    float speed = 0.1;
    float turn_angle = 0.1;

    for (int row = 0; row < img.height; row++) {
        for (int col = 0; col < img.step - 3; col += 3) {
            int index = row * img.step + col;
            int red = img.data[index + 0];
            int green = img.data[index + 1];
            int blue = img.data[index + 2];
                        
            if (red == white_pixel && green == white_pixel && blue == white_pixel) {
                // the white ball was found, determine the ball's location and drive to towards it
                
                ROS_INFO_STREAM("image data at coordinate (" + std::to_string(row) + ", " + std::to_string(col) + "), index (" + std::to_string(index) + ") =  RGB(" + std::to_string(red) + ", " + std::to_string(green) + ", " + std::to_string(blue) + ")");
                
                if (col < segment_size) {
                    // the white ball is on the left
                    
                    ROS_INFO_STREAM("Turning left");
                    drive_robot(speed, turn_angle);
                } else if (col < (segment_size * 2)) {
                    // the white ball is in the middle
                    
                    ROS_INFO_STREAM("Going straight");
                    drive_robot(speed, 0.0);
                } else {
                    // the white ball is on the right
                    
                    ROS_INFO_STREAM("Turning right");
                    drive_robot(speed, -turn_angle);
                }
                
                return;
            }
        }
    }
    
    // the ball was not found, stop the robot
    ROS_INFO_STREAM("Stopping");
    drive_robot(0, 0);
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

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <iostream>

ros::Publisher velocity_pub_; // Publisher for velocity
ros::Publisher error_pub_;    // Publisher for error

geometry_msgs::PoseStamped kyle_pose_; // Pose of top turtle
ros::Time last_msg_time_;              // Last time callback was called (to calculate delta t)

double integral_, pre_error_;

// PID Global Variables
double Kp_ = 1;  // increase effort when error increases
double Ki_ = 1;  // when error persists for a long time, increase effort
double Kd_ = 1; // faster decrease in error = more slow (decr < 0)

/**
 * Callback for Kyle (top turtle). Saves the position of the top turtle into the global
 * variable kyle_pose_
 * @param msg message containing Kyle's pose
 */
void kylePoseCallback(geometry_msgs::PoseStamped msg)
{
    // IMPLEMENT!
    kyle_pose_ = msg;
}


/**
 * Uses PID terms (Kp, Ki, Kd) to comput output given error and delta time
 * @param error error feed into PID (goal.x - current.x)
 * @param dt delta time from last update to this one
 */
double pid(double error, double dt) {
    // TODO: why tf does the turtle pass its goal
    // this fixes things but it's basically bang-bang in reverse
    // std::cout << error << std::endl;
    // if (error < 0) {
    //     std::cout << "!!!" << std::endl;
    //     return 0;
    // }

    // IMPLEMENT!
    // Proportional term
    double prop_term = Kp_*error;

    // Integral term
    integral_ += error*dt;
    // std::cout << integral_ << std::endl;
    double integral_term = Ki_*integral_;

    // Derivative term
    double deriv_term = Kd_*(error-pre_error_)/dt;

    // Calculate total output
    return prop_term+integral_term+deriv_term;
}


/**
 * Callback for Oswin (bottom turtle). Calculates the error in x between the two turtles,
 * and then uses a PID Controller to calculate a control to publish
 * @param msg message containing Oswin's pose
 */
void oswinPoseCallback(geometry_msgs::PoseStamped msg)
{
    // IMPLEMENT!
    ros::Time now = ros::Time::now();

    // Check if last_msg_time_ is populated (not first callback call)
    if (last_msg_time_.sec != 0) {
        // Calculate error in x between top and bottom turtles and the delta time
        geometry_msgs::Pose o_pose = msg.pose;
        geometry_msgs::Point o_pos = o_pose.position;
        double o_x = o_pos.x;

        geometry_msgs::Pose k_pose = kyle_pose_.pose;
        geometry_msgs::Point k_pos = k_pose.position;
        double k_x = k_pos.x;

        double error = k_x - o_x;
        // find dt for integral controller
        double dt = now.toSec(); - last_msg_time_.toSec();

        // Call PID function to get controls
        double output_x = pid(error, dt);

        // Save message time in last_msg_time_
        // below

        // publish a geometry_msgs::Twist message so that the turtle will move
        geometry_msgs::Twist out_velocity;
        out_velocity.linear.x = output_x;
        velocity_pub_.publish(out_velocity);

        // publish a std_msgs::Float64 message to be able to graph the error in rqt_plot
        std_msgs::Float64 out_error;
        out_error.data = error;
        error_pub_.publish(out_error);

        // save past error for derivative controller
        pre_error_ = error;
    }
    last_msg_time_ = now;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");

    // IMPLEMENT!

    // Create global nodehandle
    ros::NodeHandle nh;

    // Advertise "/oswin/velocity" to control the bottom turtle and "/error" for visualization
    velocity_pub_ = nh.advertise<geometry_msgs::Twist>("/oswin/velocity", 1);
    error_pub_ = nh.advertise<std_msgs::Float64>("/error", 1);

    // Subscriber to both ground truth topics to get their positions
    ros::Subscriber kyle_sub = nh.subscribe("/kyle/ground_truth", 1, &kylePoseCallback);
    ros::Subscriber oswin_sub = nh.subscribe("/oswin/ground_truth", 1, &oswinPoseCallback);

    // Don't forget to call ros::spin() to let ros do things behind the scenes and call your callback
    // functions when it receives a new message!
    ros::spin();
}

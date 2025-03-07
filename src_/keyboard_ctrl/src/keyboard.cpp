#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sys/select.h>
#include <signal.h>

#define WAFFLE_MAX_LIN_VEL 0.26
#define WAFFLE_MAX_ANG_VEL 1.82
#define LIN_VEL_STEP_SIZE 0.05
#define ANG_VEL_STEP_SIZE 0.1

bool running = true;

void signalHandler(int signum)
{
    running = false;
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
}

std::string msg = R"(
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
)";

char getKey()
{
    struct termios oldt, newt;
    char c = 0;
    struct timeval timeout;
    fd_set readfds;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms 

    if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0)
    {
        read(STDIN_FILENO, &c, 1);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

double constrain(double input, double low, double high)
{
    if (input < low) return low;
    if (input > high) return high;
    return input;
}

double checkLinearLimitVelocity(double vel)
{
    return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
}

double checkAngularLimitVelocity(double vel)
{
    return constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "keyboard_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Duration(0.5).sleep(); 

    double target_linear_vel = 0.0;
    double target_angular_vel = 0.0;

    std::cout << msg;

    while (ros::ok() && running)
    {
        char key = getKey();
        if (key == 'w')
        {
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
            std::cout << "currently:\tlinear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << "\n";
        }
        else if (key == 'x')
        {
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
            std::cout << "currently:\tlinear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << "\n";
        }
        else if (key == 'a')
        {
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE);
            std::cout << "currently:\tlinear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << "\n";
        }
        else if (key == 'd')
        {
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE);
            std::cout << "currently:\tlinear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << "\n";
        }
        else if (key == ' ' || key == 's')
        {
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            std::cout << "currently:\tlinear vel " << target_linear_vel << "\t angular vel " << target_angular_vel << "\n";
        }
        else if (key == 3) // ASCII value 3 == CTRL-C
        {
            std::cout << "Exit" << std::endl;
            break;
        }

        geometry_msgs::Twist twist;
        twist.linear.x = target_linear_vel;
        twist.angular.z = target_angular_vel;
        pub.publish(twist);

        ros::spinOnce();
    }

    // stop robot before exit
    geometry_msgs::Twist twist;
    pub.publish(twist);

    return 0;
}

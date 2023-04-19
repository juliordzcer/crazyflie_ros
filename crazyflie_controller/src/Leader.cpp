#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <cmath>
#include <vector>

#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_pubLu()
        , m_listener()
        , m_pidNUX(
            get(n, "PIDs/NUX/kp"),
            get(n, "PIDs/NUX/kd"),
            get(n, "PIDs/NUX/ki"),
            get(n, "PIDs/NUX/minOutput"),
            get(n, "PIDs/NUX/maxOutput"),
            get(n, "PIDs/NUX/integratorMin"),
            get(n, "PIDs/NUX/integratorMax"),
            "NUx")
        , m_pidNUY(
            get(n, "PIDs/NUY/kp"),
            get(n, "PIDs/NUY/kd"),
            get(n, "PIDs/NUY/ki"),
            get(n, "PIDs/NUY/minOutput"),
            get(n, "PIDs/NUY/maxOutput"),
            get(n, "PIDs/NUY/integratorMin"),
            get(n, "PIDs/NUY/integratorMax"),
            "NUy")
        , m_pidNUZ(
            get(n, "PIDs/NUZ/kp"),
            get(n, "PIDs/NUZ/kd"),
            get(n, "PIDs/NUZ/ki"),
            get(n, "PIDs/NUZ/minOutput"),
            get(n, "PIDs/NUZ/maxOutput"),
            get(n, "PIDs/NUZ/integratorMin"),
            get(n, "PIDs/NUZ/integratorMax"),
            "NUz")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_statecameras()
        , m_goalvel()
        , m_goalacc()
        , m_subscribeGoal()
        , m_subscribeStateCameras()
        , m_subscribeGoalVel()
        , m_subscribeGoalAcc()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_height(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_pubLu = nh.advertise<std_msgs::Float32>("leader_u", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_subscribeStateCameras = nh.subscribe("vrpn_client_node/crazyflie/pose", 1, &Controller::stateCameras, this);
        m_subscribeGoalVel = nh.subscribe("goalvel", 1, &Controller::goalvelChanged, this);
        m_subscribeGoalAcc = nh.subscribe("goalacc", 1, &Controller::goalaccChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }
    void stateCameras(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_statecameras= *msg;
    }
    void goalvelChanged(
        const geometry_msgs::Twist::ConstPtr& msg)
    {
        m_goalvel = *msg;
    }
    void goalaccChanged(
        const geometry_msgs::Twist::ConstPtr& msg)
    {
        m_goalacc = *msg;
    }


    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        m_startZ = 0;

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidNUX.reset();
        m_pidNUY.reset();
        m_pidNUZ.reset();
        m_pidYaw.reset();
    }


    float rad2deg(float radianes) {
        return radianes * 180 / M_PI;
    }
    float deg2rad(float deg) {
        return deg *  M_PI / 180 ;
    }

    float sign(float n)
    {
    if(n > 0)
        return 1.0;
    else if(n < 0)
        return -1.0;
    else
        return 0.0;
    }

    float calculate_rpm(float  thrust_newtons) {

    float thrust_gramos = thrust_newtons / 0.00980665f;
    float a = 1.0942e-07f;
    float b = -2.1059e-04f;
    float c = 1.5417e-01f;
    float discriminante = powf(b, 2.0f) - 4.0f * a * (c - fabsf(thrust_gramos));
    
    return (-b + sqrtf(discriminante)) / (2.0f * a) * sign(thrust_gramos);
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                if (m_statecameras.pose.position.z > 0.05 || m_thrust > 16000)
                {
                    pidReset();
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }
            }
            break;

            case Landing:
            {
                m_thrust -= 1000 * dt;
                if (m_thrust < 0 || m_statecameras.pose.position.z < 0.05 ) 
                {
                    m_thrust = 0;
                }
            }
            break;

            // intentional fall-thru

            case Automatic: {

            tf::Quaternion q_target(
                m_goal.pose.orientation.x,
                m_goal.pose.orientation.y,
                m_goal.pose.orientation.z,
                m_goal.pose.orientation.w
            );
            double rolld, pitchd, yawd;
            tf::Matrix3x3(q_target).getRPY(rolld, pitchd, yawd);

            tf::Quaternion q_target_drone(
                m_statecameras.pose.orientation.x,
                m_statecameras.pose.orientation.y,
                m_statecameras.pose.orientation.z,
                m_statecameras.pose.orientation.w
            );
            double roll, pitch, yaw;
            tf::Matrix3x3(q_target_drone).getRPY(roll, pitch, yaw);

            float NUXS = (m_pidNUX.update(m_statecameras.pose.position.x, m_goal.pose.position.x)) + m_goalacc.linear.x;
            float NUYS = (m_pidNUY.update(m_statecameras.pose.position.y, m_goal.pose.position.y)) + m_goalacc.linear.y;
            float NUZS = (m_pidNUZ.update(m_statecameras.pose.position.z, m_goal.pose.position.z)) + m_goalacc.linear.z;

            float m = 0.032f;
            float u = sqrtf(powf(NUXS, 2.0f) + powf(NUYS, 2.0f) + powf((NUZS + 9.81f), 2.0f)) * m;
            u = std::max(std::min(u, 5.0f), 0.0f);
            float u_rpm = std::max(std::min(calculate_rpm(u), 60000.0f), 10000.0f);
            float phi = asinf((NUXS * sin(yawd) - NUYS * cos(yawd))*( m / u )) ;
            float theta = atanf((NUXS * cos(yawd) + NUYS * sin(yawd)) / (NUZS + 9.81));
            
            std_msgs::Float32 msg_u;
            msg_u.data = u;
            m_pubLu.publish(msg_u);

            if (m_statecameras.pose.position.z < 0.05)
            {
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = u_rpm;
                msg.angular.z = 0;
                m_pubNav.publish(msg);
            }
            else
            { 
                geometry_msgs::Twist msg;
                msg.linear.x = std::max(std::min(rad2deg(theta), 10.0f), -10.0f);
                msg.linear.y = std::max(std::min(rad2deg(phi), 10.0f), -10.0f);
                msg.linear.z = u_rpm;
                msg.angular.z = m_pidYaw.update(yaw, yawd);
                m_pubNav.publish(msg);
            }

            m_thrust = u_rpm;


        }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    ros::Publisher m_pubLu;
    tf::TransformListener m_listener;
    PID m_pidNUX;
    PID m_pidNUY;
    PID m_pidNUZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    geometry_msgs::PoseStamped m_statecameras;
    geometry_msgs::Twist m_goalvel;
    geometry_msgs::Twist m_goalacc;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeStateCameras;
    ros::Subscriber m_subscribeGoalVel;
    ros::Subscriber m_subscribeGoalAcc;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_height;
    float m_startZ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}

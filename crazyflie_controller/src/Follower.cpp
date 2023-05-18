

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

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
        , bool_pub()
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
        , m_gamma()
        , m_goalacc()
        , m_subscribeGoal()
        , m_subscribeGamma()
        , m_subscribeGoalAcc()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_height(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        bool_pub = nh.advertise<std_msgs::Bool>("boolean_topic", 10);
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_subscribeGamma = nh.subscribe("info_leader", 1, &Controller::gammaChanged, this);
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
    void gammaChanged(
        const geometry_msgs::Twist::ConstPtr& msg)
    {
        m_gamma = *msg;
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

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

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


    // Definir la función que convierte radianes a grados
    float rad2deg(float radianes) {
        // Aplicar la fórmula y retornar el resultado
        return radianes * 180 / M_PI;
    }
    float deg2rad(float deg) {
        // Aplicar la fórmula y retornar el resultado
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
                tf::StampedTransform transform;
                std_msgs::Bool msg;
                msg.data = true; 
                bool_pub.publish(msg);
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 15000)
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
                    m_thrust = 38000;

                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);

                    tf::StampedTransform transform;
                    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                    if (transform.getOrigin().z() <= m_startZ + 0.05)
                    {
                        m_state = Idle;
                        geometry_msgs::Twist msg;
                        m_pubNav.publish(msg);
                    }
                }
                break;


                // intentional fall-thru
                case Automatic: {

                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() >= 0.15)
                {
                    std_msgs::Bool msg;
                    msg.data = false; 
                    bool_pub.publish(msg);
                }
                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                tfScalar roll_d, pitch_d, yaw_d;
                tf::Matrix3x3(
                    tf::Quaternion(
                        m_goal.pose.orientation.x,
                        m_goal.pose.orientation.y,
                        m_goal.pose.orientation.z,
                        m_goal.pose.orientation.w
                    )).getRPY(roll_d, pitch_d, yaw_d);

                float NUXS = (m_pidNUX.update(0.0, targetDrone.pose.position.x)) + m_gamma.linear.x;
                float NUYS = (m_pidNUY.update(0.0, targetDrone.pose.position.y)) + m_gamma.linear.y;
                float NUZS = (m_pidNUZ.update(0.0, targetDrone.pose.position.z)) + m_gamma.linear.z;

                float m = 0.032;
                float u = sqrt(pow(NUXS, 2) + pow(NUYS, 2) + pow((NUZS + 9.81), 2)) * m;
                float u_rpm = std::max(std::min(calculate_rpm(u), 60000.0f), 10000.0f);
                
                float phi = asin((NUXS * sin(yaw_d) - NUYS * cos(yaw_d))*( m / u )) ;
                float theta = atan((NUXS * cos(yaw_d) + NUYS * sin(yaw_d)) / (NUZS + 9.81));

    
                geometry_msgs::Twist msg;
                msg.linear.x = std::max(std::min(rad2deg(theta), 10.0f), -10.0f);
                msg.linear.y = std::max(std::min(rad2deg(phi), 10.0f), -10.0f);
                msg.linear.z = u_rpm;
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
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
    ros::Publisher bool_pub;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidNUX;
    PID m_pidNUY;
    PID m_pidNUZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    geometry_msgs::Twist m_gamma;
    geometry_msgs::Twist m_goalacc;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeGamma;
    ros::Subscriber m_subscribeGoalAcc;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
    float m_height;
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

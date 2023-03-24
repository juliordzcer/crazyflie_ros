

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <cmath>
#include <vector>

#include "pid.hpp"

#define CLAMP(x, a, b) std::min(std::max((x), (a)), (b))

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
        , m_goalvel()
        , m_goalacc()
        , m_subscribeGoal()
        , m_subscribeGoalVel()
        , m_subscribeGoalAcc()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
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

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.01 || m_thrust > 20000)
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

        case Automatic: {
            constexpr float a = 0.109e-6;
            constexpr float b = -210.59e-6;
            constexpr float c = 0.1517;
            constexpr float m = 0.032;
            constexpr float g = 9.80665;

            tf::StampedTransform transform;

            try {
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
            } catch (tf::TransformException &ex) {
                ROS_ERROR_STREAM("Failed to lookup transform: " << ex.what());
                break;
            }

            geometry_msgs::PoseStamped targetWorld;
            targetWorld.header.stamp = transform.stamp_;
            targetWorld.header.frame_id = m_worldFrame;
            targetWorld.pose = m_goal.pose;

            geometry_msgs::PoseStamped targetDrone;

            try {
                m_listener.transformPose(m_frame, targetWorld, targetDrone);
            } catch (tf::TransformException &ex) {
                ROS_ERROR_STREAM("Failed to transform pose: " << ex.what());
                break;
            }

            tfScalar roll, pitch, yaw;
            tf::Matrix3x3(tf::Quaternion(
                            targetDrone.pose.orientation.x,
                            targetDrone.pose.orientation.y,
                            targetDrone.pose.orientation.z,
                            targetDrone.pose.orientation.w)).getRPY(roll, pitch, yaw);

            const float NUZS = (m_pidNUZ.update(0.0, targetDrone.pose.position.z)) + m_goalacc.linear.z;
            const float NUXS = (m_pidNUX.update(0.0, targetDrone.pose.position.x)) + m_goalacc.linear.x;
            const float NUYS = (m_pidNUY.update(0.0, targetDrone.pose.position.y)) + m_goalacc.linear.y;

            if (!std::isfinite(NUZS) || !std::isfinite(NUXS) || !std::isfinite(NUYS)) {
                ROS_ERROR_STREAM("Invalid control inputs: NUZS=" << NUZS << ", NUXS=" << NUXS << ", NUYS=" << NUYS);
                break;
            }

            const float u = std::sqrt(std::pow(NUXS, 2) + std::pow(NUYS, 2) + std::pow(NUZS + g, 2)) * m;
            const float u_gramos = u / (g / 1000);
            const float u_rpm = (std::sqrt(4 * a * (u_gramos - c) + std::pow(b, 2)) - b) / (2 * a);
            const float phi = std::asin((NUXS / u) * std::sin(yaw) - (NUYS / u) * std::cos(yaw));
            const float theta = std::atan2((NUXS * std::cos(yaw) + NUYS * std::sin(yaw)), (NUZS + g));

            if (!std::isfinite(phi) || !std::isfinite(theta) || !std::isfinite(u_rpm)) {
                ROS_ERROR_STREAM("Invalid control outputs: phi=" << phi << ", theta=" << theta << ", u_rpm=" << u_rpm);
                break;
            }

            geometry_msgs::Twist msg;
            msg.linear.x = CLAMP(rad2deg(theta), -10.0f, 10.0f);
            msg.linear.y = CLAMP(rad2deg(phi), -10.0f, 10.0f);
            msg.linear.z = CLAMP(u_rpm, 10000.0f, 60000.0f);
            msg.angular.z = m_pidYaw.update(0.0, yaw);
            m_pubNav.publish(msg);
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
    tf::TransformListener m_listener;
    PID m_pidNUX;
    PID m_pidNUY;
    PID m_pidNUZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    geometry_msgs::Twist m_goalvel;
    geometry_msgs::Twist m_goalacc;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeGoalVel;
    ros::Subscriber m_subscribeGoalAcc;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
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

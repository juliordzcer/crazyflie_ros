#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>

#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/NotifySetpointsStop.h"
#undef major
#undef minor
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/VelocityWorld.h"
#include "std_srvs/Empty.h"
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>

//#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <crazyflie_cpp/Crazyflie.h>

// debug test
#include <signal.h>
#include <csignal> // or C++ style alternative

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Object tracker
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>

#include <fstream>
#include <future>
#include <mutex>
#include <wordexp.h> // tilde expansion

// Carpetas no utilizadas en CrazySwarm

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/RemoveCrazyflie.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/sendPacket.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/crtpPacket.h"
#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <string>
#include <map>


constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}

//Experimento 1/nov/2022


class ROSLogger : public Logger
{
public:
  ROSLogger()
    : Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }

  virtual void warning(const std::string& msg)
  {
    ROS_WARN("%s", msg.c_str());
  }

  virtual void error(const std::string& msg)
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

static ROSLogger rosLogger;

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    int id,
    const std::string& type,
    const std::vector<crazyswarm::LogBlock>& log_blocks,
    ros::CallbackQueue& queue)
    : m_tf_prefix(tf_prefix)
    , m_cf(
      link_uri,
      rosLogger,
      std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1))
    , m_frame(frame)
    , m_worldFrame(worldFrame)
    , m_id(id)
    , m_type(type)
    , m_serviceUpdateParams()
    , m_serviceUploadTrajectory()
    , m_serviceStartTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceGoTo()
    , m_serviceSetGroupMask()
    , m_serviceNotifySetpointsStop()
    , m_logBlocks(log_blocks)
    , m_initializedPosition(false)
  {
    ros::NodeHandle nl("~");
    nl.param("enable_logging", m_enableLogging, false);
    nl.param("enable_logging_pose", m_enableLoggingPose, false);
    nl.param("enable_parameters", m_enableParameters, true);
    nl.param("force_no_cache", m_forceNoCache, false);

    ros::NodeHandle n;
    n.setCallbackQueue(&queue);
    m_serviceUploadTrajectory    = n.advertiseService(tf_prefix + "/upload_trajectory",     &CrazyflieROS::uploadTrajectory,    this);
    m_serviceStartTrajectory     = n.advertiseService(tf_prefix + "/start_trajectory",      &CrazyflieROS::startTrajectory,     this);
    m_serviceTakeoff             = n.advertiseService(tf_prefix + "/takeoff",               &CrazyflieROS::takeoff,             this);
    m_serviceLand                = n.advertiseService(tf_prefix + "/land",                  &CrazyflieROS::land,                this);
    m_serviceGoTo                = n.advertiseService(tf_prefix + "/go_to",                 &CrazyflieROS::goTo,                this);
    m_serviceSetGroupMask        = n.advertiseService(tf_prefix + "/set_group_mask",        &CrazyflieROS::setGroupMask,        this);
    m_serviceNotifySetpointsStop = n.advertiseService(tf_prefix + "/notify_setpoints_stop", &CrazyflieROS::notifySetpointsStop, this);

    m_subscribeCmdVel           = n.subscribe(tf_prefix   + "/cmd_vel",            1, &CrazyflieROS::cmdVelChanged,            this);
    m_subscribeCmdPosition      = n.subscribe(tf_prefix   + "/cmd_position",       1, &CrazyflieROS::cmdPositionSetpoint,      this);
    m_subscribeCmdFullState     = n.subscribe(tf_prefix   + "/cmd_full_state",     1, &CrazyflieROS::cmdFullStateSetpoint,     this);
    m_subscribeCmdVelocityWorld = n.subscribe(tf_prefix   + "/cmd_velocity_world", 1, &CrazyflieROS::cmdVelocityWorldSetpoint, this);
    m_subscribeCmdStop          = n.subscribe(m_tf_prefix + "/cmd_stop",           1, &CrazyflieROS::cmdStop,                  this);
    m_subscribeCmdHover         = n.subscribe(m_tf_prefix + "/cmd_hover",          1, &CrazyflieROS::cmdHoverSetpoint,         this);

    if (m_enableLogging) {
      m_logFile.open("logcf" + std::to_string(id) + ".csv");
      m_logFile << "time,";
      for (auto& logBlock : m_logBlocks) {
        m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
        for (const auto& variableName : logBlock.variables) {
          m_logFile << variableName << ",";
        }
      }
      m_logFile << std::endl;

      if (m_enableLoggingPose) {
        m_pubPose = n.advertise<geometry_msgs::PoseStamped>(m_tf_prefix + "/pose", 10);
      }
    }
  }

  ~CrazyflieROS()
  {
    m_logBlocks.clear();
    m_logBlocksGeneric.clear();
    m_logFile.close();
  }

  const std::string& frame() const {
    return m_frame;
  }

  const int id() const {
    return m_id;
  }

  const std::string& type() const {
    return m_type;
  }

  void sendPing() {
    m_cf.sendPing();
  }

public:

  template<class T, class U>
  void updateParam(uint16_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.addSetParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("[%s] Update parameters", m_frame.c_str());
    m_cf.startSetParamRequest();
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    m_cf.setRequestedParams();
    return true;
  }


  bool uploadTrajectory(
    crazyflie_driver::UploadTrajectory::Request& req,
    crazyflie_driver::UploadTrajectory::Response& res)
  {
    ROS_INFO("[%s] Upload trajectory", m_frame.c_str());

    std::vector<Crazyflie::poly4d> pieces(req.pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i) {
      if (   req.pieces[i].poly_x.size() != 8
          || req.pieces[i].poly_y.size() != 8
          || req.pieces[i].poly_z.size() != 8
          || req.pieces[i].poly_yaw.size() != 8) {
        ROS_FATAL("Wrong number of pieces!");
        return false;
      }
      pieces[i].duration = req.pieces[i].duration.toSec();
      for (size_t j = 0; j < 8; ++j) {
        pieces[i].p[0][j] = req.pieces[i].poly_x[j];
        pieces[i].p[1][j] = req.pieces[i].poly_y[j];
        pieces[i].p[2][j] = req.pieces[i].poly_z[j];
        pieces[i].p[3][j] = req.pieces[i].poly_yaw[j];
      }
    }
    m_cf.uploadTrajectory(req.trajectoryId, req.pieceOffset, pieces);

    ROS_INFO("[%s] Uploaded trajectory", m_frame.c_str());


    return true;
  }

  bool startTrajectory(
    crazyflie_driver::StartTrajectory::Request& req,
    crazyflie_driver::StartTrajectory::Response& res)
  {
    ROS_INFO("[%s] Start trajectory", m_frame.c_str());

    m_cf.startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.relative, req.groupMask);

    return true;
  }

  bool notifySetpointsStop(
    crazyflie_driver::NotifySetpointsStop::Request& req,
    crazyflie_driver::NotifySetpointsStop::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "NotifySetpointsStop requested");
    m_cf.notifySetpointsStop(req.remainValidMillisecs);
    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO("[%s] Takeoff", m_frame.c_str());

    m_cf.takeoff(req.height, req.duration.toSec(), req.groupMask);

    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO("[%s] Land", m_frame.c_str());

    m_cf.land(req.height, req.duration.toSec(), req.groupMask);

    return true;
  }

  bool goTo(
    crazyflie_driver::GoTo::Request& req,
    crazyflie_driver::GoTo::Response& res)
  {
    ROS_INFO("[%s] GoTo", m_frame.c_str());

    m_cf.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.relative, req.groupMask);

    return true;
  }

  bool setGroupMask(
    crazyflie_driver::SetGroupMask::Request& req,
    crazyflie_driver::SetGroupMask::Response& res)
  {
    ROS_INFO("[%s] Set Group Mask", m_frame.c_str());

    m_cf.setGroupMask(req.groupMask);

    return true;
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    // if (!m_isEmergency) {
      float roll = msg->linear.y;
      float pitch = -msg->linear.x;
      float yawrate = msg->angular.z;
      uint16_t thrust = (uint16_t)msg->linear.z;

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      // ROS_INFO("cmdVel %f %f %f %d (%f)", roll, pitch, yawrate, thrust, msg->linear.z);
      // m_sentSetpoint = true;
    // }
  }

  void cmdPositionSetpoint(
    const crazyflie_driver::Position::ConstPtr& msg)
  {
    // if(!m_isEmergency) {
      float x = msg->x;
      float y = msg->y;
      float z = msg->z;
      float yaw = msg->yaw;

      m_cf.sendPositionSetpoint(x, y, z, yaw);
      // m_sentSetpoint = true;
    // }
  }

  void cmdFullStateSetpoint(
    const crazyflie_driver::FullState::ConstPtr& msg)
  {
    //ROS_INFO("got a full state setpoint");
    // if (!m_isEmergency) {
      float x = msg->pose.position.x;
      float y = msg->pose.position.y;
      float z = msg->pose.position.z;
      float vx = msg->twist.linear.x;
      float vy = msg->twist.linear.y;
      float vz = msg->twist.linear.z;
      float ax = msg->acc.x;
      float ay = msg->acc.y;
      float az = msg->acc.z;

      float qx = msg->pose.orientation.x;
      float qy = msg->pose.orientation.y;
      float qz = msg->pose.orientation.z;
      float qw = msg->pose.orientation.w;
      float rollRate = msg->twist.angular.x;
      float pitchRate = msg->twist.angular.y;
      float yawRate = msg->twist.angular.z;

      m_cf.sendFullStateSetpoint(
        x, y, z,
        vx, vy, vz,
        ax, ay, az,
        qx, qy, qz, qw,
        rollRate, pitchRate, yawRate);
      // m_sentSetpoint = true;
      //ROS_INFO("set a full state setpoint");
    // }
  }

  void cmdHoverSetpoint(const crazyflie_driver::Hover::ConstPtr& msg){
     //ROS_INFO("got a hover setpoint");
      float vx = msg->vx;
      float vy = msg->vy;
      float yawRate = msg->yawrate;
      float zDistance = msg->zDistance;

      m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
      //ROS_INFO("set a hover setpoint");

  }
  void cmdVelocityWorldSetpoint(
    const crazyflie_driver::VelocityWorld::ConstPtr& msg)
  {
    // ROS_INFO("got a velocity world setpoint");
    // if (!m_isEmergency) {
      float x = msg->vel.x;
      float y = msg->vel.y;
      float z = msg->vel.z;
      float yawRate = msg->yawRate;

      m_cf.sendVelocityWorldSetpoint(
        x, y, z, yawRate);
      // m_sentSetpoint = true;
      // ROS_INFO("set a velocity world setpoint");
    // }
  }

  void cmdStop(
    const std_msgs::Empty::ConstPtr& msg)
  {
     //ROS_INFO("got a stop setpoint");
    // if (!m_isEmergency) {
      m_cf.sendStop();
      // m_sentSetpoint = true;
      //ROS_INFO("set a stop setpoint");
    // }
  }

  void run(
    ros::CallbackQueue& queue)
  {
    // m_cf.reboot();
    // m_cf.syson();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto start = std::chrono::system_clock::now();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    m_cf.logReset();

    int numParams = 0;
    if (m_enableParameters)
    {
      ROS_INFO("[%s] Requesting parameters...", m_frame.c_str());
      m_cf.requestParamToc(m_forceNoCache);
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
        ++numParams;
      }
      ros::NodeHandle n;
      n.setCallbackQueue(&queue);
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }
    auto end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds1 = end1-start;
    ROS_INFO("[%s] reqParamTOC: %f s (%d params)", m_frame.c_str(), elapsedSeconds1.count(), numParams);

    // Logging
    if (m_enableLogging) {
      ROS_INFO("[%s] Requesting logging variables...", m_frame.c_str());
      m_cf.requestLogToc(m_forceNoCache);
      auto end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds2 = end2-end1;
      ROS_INFO("[%s] reqLogTOC: %f s", m_frame.c_str(), elapsedSeconds2.count());

      m_logBlocksGeneric.resize(m_logBlocks.size());
      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        m_logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        m_logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }
      auto end3 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds3 = end3-end2;
      ROS_INFO("[%s] logBlocks: %f s", m_frame.c_str(), elapsedSeconds1.count());

      if (m_enableLoggingPose) {
        std::function<void(uint32_t, logPose*)> cb = std::bind(&CrazyflieROS::onPoseData, this, std::placeholders::_1, std::placeholders::_2);

        m_logBlockPose.reset(new LogBlock<logPose>(
          &m_cf,{
            {"stateEstimate", "x"},
            {"stateEstimate", "y"},
            {"stateEstimate", "z"},
            {"stateEstimateZ", "quat"}
          }, cb));
        m_logBlockPose->start(10); // 100ms
      }
    }

    ROS_INFO("Requesting memories...");
    m_cf.requestMemoryToc();

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("[%s] Ready. Elapsed: %f s", m_frame.c_str(), elapsedSeconds.count());
  }

  const Crazyflie::ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const
  {
    return m_cf.getParamTocEntry(group, name);
  }

  void initializePositionIfNeeded(float x, float y, float z)
  {
    if (m_initializedPosition) {
      return;
    }

    m_cf.startSetParamRequest();
    auto entry = m_cf.getParamTocEntry("kalman", "initialX");
    m_cf.addSetParam(entry->id, x);
    entry = m_cf.getParamTocEntry("kalman", "initialY");
    m_cf.addSetParam(entry->id, y);
    entry = m_cf.getParamTocEntry("kalman", "initialZ");
    m_cf.addSetParam(entry->id, z);
    m_cf.setRequestedParams();

    entry = m_cf.getParamTocEntry("kalman", "resetEstimation");
    m_cf.setParam<uint8_t>(entry->id, 1);

    // kalmanUSC might not be part of the firmware
    entry = m_cf.getParamTocEntry("kalmanUSC", "resetEstimation");
    if (entry) {
      m_cf.startSetParamRequest();
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialX");
      m_cf.addSetParam(entry->id, x);
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialY");
      m_cf.addSetParam(entry->id, y);
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialZ");
      m_cf.addSetParam(entry->id, z);
      m_cf.setRequestedParams();

      entry = m_cf.getParamTocEntry("kalmanUSC", "resetEstimation");
      m_cf.setParam<uint8_t>(entry->id, 1);
    }

    m_initializedPosition = true;
  }

private:
  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

private:

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("[%s] Link Quality low (%f)", m_frame.c_str(), linkQuality);
      }
  }

  void onConsole(const char* msg) {
    m_messageBuffer += msg;
    size_t pos = m_messageBuffer.find('\n');
    if (pos != std::string::npos) {
      m_messageBuffer[pos] = 0;
      ROS_INFO("[%s] %s", m_frame.c_str(), m_messageBuffer.c_str());
      m_messageBuffer.erase(0, pos+1);
    }
  }

  void onPoseData(uint32_t time_in_ms, logPose* data) {
    if (m_enableLoggingPose) {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      m_pubPose.publish(msg);


      tf::Transform tftransform;
      tftransform.setOrigin(tf::Vector3(data->x, data->y, data->z));
      tftransform.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));
      m_br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", frame()));
    }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    msg.header.stamp = ros::Time(time_in_ms/1000.0);
    msg.values = *values;

    m_logFile << time_in_ms / 1000.0 << ",";
    for (const auto& value : *values) {
      m_logFile << value << ",";
    }
    m_logFile << std::endl;

    pub->publish(msg);
  }

private:
  std::string m_tf_prefix;
  Crazyflie m_cf;
  std::string m_frame;
  std::string m_worldFrame;
  bool m_enableParameters;
  bool m_enableLogging;
  bool m_enableLoggingPose;
  int m_id;
  std::string m_type;

  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceUploadTrajectory;
  ros::ServiceServer m_serviceStartTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceGoTo;
  ros::ServiceServer m_serviceSetGroupMask;
  ros::ServiceServer m_serviceNotifySetpointsStop;

  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeCmdPosition;
  ros::Subscriber m_subscribeCmdFullState;
  ros::Subscriber m_subscribeCmdVelocityWorld;
  ros::Subscriber m_subscribeCmdStop;

  ros::Subscriber m_subscribeCmdHover; // Hover vel subscriber

  tf::TransformBroadcaster m_br;

  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  std::vector<std::unique_ptr<LogBlockGeneric> > m_logBlocksGeneric;

  ros::Subscriber m_subscribeJoy;

  ros::Publisher m_pubPose;
  std::unique_ptr<LogBlock<logPose>> m_logBlockPose;

  std::ofstream m_logFile;
  bool m_forceNoCache;
  bool m_initializedPosition;
  std::string m_messageBuffer;
};


// handles a group of Crazyflies, which share a radio
class CrazyflieGroup
{
public:
  struct latency
  {
    double objectTracking;
    double broadcasting;
  };

  CrazyflieGroup(
    const std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations,
    const std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMarkers,
    std::map<std::string, libmotioncapture::RigidBody>* pMocapRigidBodies,
    int radio,
    int channel,
    bool useMotionCaptureObjectTracking,
    const std::vector<crazyswarm::LogBlock>& logBlocks,
    std::string interactiveObject,
    bool writeCSVs,
    bool sendPositionOnly
    )
    : m_cfs()
    , m_tracker(nullptr)
    , m_radio(radio)
    , m_pMarkers(pMarkers)
    , m_pMocapRigidBodies(pMocapRigidBodies)
    , m_slowQueue()
    , m_cfbc("radio://" + std::to_string(radio) + "/" + std::to_string(channel) + "/2M/FFE7E7E7E7")
    , m_isEmergency(false)
    , m_useMotionCaptureObjectTracking(useMotionCaptureObjectTracking)
    , m_br()
    , m_interactiveObject(interactiveObject)
    , m_sendPositionOnly(sendPositionOnly)
    , m_outputCSVs()
    , m_phase(0)
    , m_phaseStart()
  {
    std::vector<libobjecttracker::Object> objects;
    readObjects(objects, channel, logBlocks);
    m_tracker = new libobjecttracker::ObjectTracker(
      dynamicsConfigurations,
      markerConfigurations,
      objects);
    m_tracker->setLogWarningCallback(logWarn);
    if (writeCSVs) {
      m_outputCSVs.resize(m_cfs.size());
      for (auto& output : m_outputCSVs) {
        output.reset(new std::ofstream);
      }
    }
  }

  ~CrazyflieGroup()
  {
    for(auto cf : m_cfs) {
      delete cf;
    }
    delete m_tracker;
  }

  const latency& lastLatency() const {
    return m_latency;
  }

  int radio() const {
    return m_radio;
  }

  void runInteractiveObject(std::vector<CrazyflieBroadcaster::externalPose> &states)
  {
    publishRigidBody(m_interactiveObject, 0xFF, states);
  }

  void runFast()
  {
    auto stamp = std::chrono::high_resolution_clock::now();

    std::vector<CrazyflieBroadcaster::externalPose> states;

    if (!m_interactiveObject.empty()) {
      runInteractiveObject(states);
    }

    if (m_useMotionCaptureObjectTracking) {
      for (auto cf : m_cfs) {
        bool found = publishRigidBody(cf->frame(), cf->id(), states);
        if (found) {
          cf->initializePositionIfNeeded(states.back().x, states.back().y, states.back().z);
        }
      }
    } else {
      // run object tracker
      {
        auto start = std::chrono::high_resolution_clock::now();
        m_tracker->update(m_pMarkers);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        m_latency.objectTracking = elapsedSeconds.count();
      }

      for (size_t i = 0; i < m_cfs.size(); ++i) {
        if (m_tracker->objects()[i].lastTransformationValid()) {

          const Eigen::Affine3f& transform = m_tracker->objects()[i].transformation();
          Eigen::Quaternionf q(transform.rotation());
          const auto& translation = transform.translation();

          states.resize(states.size() + 1);
          states.back().id = m_cfs[i]->id();
          states.back().x = translation.x();
          states.back().y = translation.y();
          states.back().z = translation.z();
          states.back().qx = q.x();
          states.back().qy = q.y();
          states.back().qz = q.z();
          states.back().qw = q.w();

          m_cfs[i]->initializePositionIfNeeded(states.back().x, states.back().y, states.back().z);

          tf::Transform tftransform;
          tftransform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
          tftransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
          m_br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", m_cfs[i]->frame()));

          if (m_outputCSVs.size() > 0) {
            std::chrono::duration<double> tDuration = stamp - m_phaseStart;
            double t = tDuration.count();
            auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            *m_outputCSVs[i] << t << "," << states.back().x << "," << states.back().y << "," << states.back().z
                                  << "," << rpy(0) << "," << rpy(1) << "," << rpy(2) << "\n";
          }
        } else {
          std::chrono::duration<double> elapsedSeconds = stamp - m_tracker->objects()[i].lastValidTime();
          ROS_WARN("No updated pose for CF %s for %f s.",
            m_cfs[i]->frame().c_str(),
            elapsedSeconds.count());
        }
      }
    }

    {
      auto start = std::chrono::high_resolution_clock::now();
      if (!m_sendPositionOnly) {
        m_cfbc.sendExternalPoses(states);
      } else {
        std::vector<CrazyflieBroadcaster::externalPosition> positions(states.size());
        for (size_t i = 0; i < positions.size(); ++i) {
          positions[i].id = states[i].id;
          positions[i].x  = states[i].x;
          positions[i].y  = states[i].y;
          positions[i].z  = states[i].z;
        }
        m_cfbc.sendExternalPositions(positions);
      }
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsedSeconds = end-start;
      m_latency.broadcasting = elapsedSeconds.count();
    }

  }

  void runSlow()
  {
    ros::NodeHandle nl("~");
    bool enableLogging;
    nl.getParam("enable_logging", enableLogging);

    while(ros::ok() && !m_isEmergency) {
      if (enableLogging) {
        for (const auto& cf : m_cfs) {
          cf->sendPing();
        }
      }
      m_slowQueue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void emergency()
  {
    m_cfbc.emergencyStop();
    m_isEmergency = true;
  }

  void takeoff(float height, float duration, uint8_t groupMask)
  {
    // for (size_t i = 0; i < 10; ++i) {
    m_cfbc.takeoff(height, duration, groupMask);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void land(float height, float duration, uint8_t groupMask)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.land(height, duration, groupMask);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void goTo(float x, float y, float z, float yaw, float duration, uint8_t groupMask)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.goTo(x, y, z, yaw, duration, groupMask);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void startTrajectory(
    uint8_t trajectoryId,
    float timescale,
    bool reversed,
    uint8_t groupMask)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.startTrajectory(trajectoryId, timescale, reversed, groupMask);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }



  template<class T, class U>
  void updateParam(const char* group, const char* name, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cfbc.setParam<T>(group, name, (T)value);
  }

  void updateParams(
    const std::vector<std::string>& params)
  {
    for (const auto& p : params) {
      std::string ros_param = "/allcfs/" + p;
      size_t pos = p.find("/");
      std::string g(p.begin(), p.begin() + pos);
      std::string n(p.begin() + pos + 1, p.end());

      // This assumes that we can find the variable in the TOC of the first
      // CF to find the type (the actual update is done by name)
      auto entry = m_cfs.front()->getParamTocEntry(g, n);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(g.c_str(), n.c_str(), ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(g.c_str(), n.c_str(), ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", g.c_str(), n.c_str());
      }
    }
  }

private:

  bool publishRigidBody(const std::string& name, uint8_t id, std::vector<CrazyflieBroadcaster::externalPose> &states)
  {
    assert(m_pMocapRigidBodies);
    const auto& iter = m_pMocapRigidBodies->find(name);
    if (iter != m_pMocapRigidBodies->end()) {
      const auto& rigidBody = iter->second;

      states.resize(states.size() + 1);
      states.back().id = id;
      states.back().x = rigidBody.position().x();
      states.back().y = rigidBody.position().y();
      states.back().z = rigidBody.position().z();
      states.back().qx = rigidBody.rotation().x();
      states.back().qy = rigidBody.rotation().y();
      states.back().qz = rigidBody.rotation().z();
      states.back().qw = rigidBody.rotation().w();

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(
        states.back().x,
        states.back().y,
        states.back().z));
      tf::Quaternion q(
        states.back().qx,
        states.back().qy,
        states.back().qz,
        states.back().qw);
      transform.setRotation(q);
      m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
      return true;
    } else {
      ROS_WARN("No updated pose for motion capture object %s", name.c_str());
    }
    return false;
  }


  void readObjects(
    std::vector<libobjecttracker::Object>& objects,
    int channel,
    const std::vector<crazyswarm::LogBlock>& logBlocks)
  {
    // read CF config
    struct CFConfig
    {
      std::string uri;
      std::string tf_prefix;
      std::string frame;
      int idNumber;
      std::string type;
    };
    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    objects.clear();
    m_cfs.clear();
    std::vector<CFConfig> cfConfigs;
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      int ch = crazyflie["channel"];
      std::string type = crazyflie["type"];
      if (ch == channel) {
        XmlRpc::XmlRpcValue pos = crazyflie["initialPosition"];
        ROS_ASSERT(pos.getType() == XmlRpc::XmlRpcValue::TypeArray);

        std::vector<double> posVec(3);
        for (int32_t j = 0; j < pos.size(); ++j) {
          switch (pos[j].getType()) {
          case XmlRpc::XmlRpcValue::TypeDouble:
            posVec[j] = static_cast<double>(pos[j]);
            break;
          case XmlRpc::XmlRpcValue::TypeInt:
            posVec[j] = static_cast<int>(pos[j]);
            break;
          default:
            std::stringstream message;
            message << "crazyflies.yaml error:"
              " entry " << j << " of initialPosition for cf" << id <<
              " should be type int or double.";
            throw std::runtime_error(message.str().c_str());
          }
        }
        Eigen::Affine3f m;
        m = Eigen::Translation3f(posVec[0], posVec[1], posVec[2]);
        int markerConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/markerConfiguration", markerConfigurationIdx);
        int dynamicsConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/dynamicsConfiguration", dynamicsConfigurationIdx);
        std::string name = "cf" + std::to_string(id);
        objects.push_back(libobjecttracker::Object(markerConfigurationIdx, dynamicsConfigurationIdx, m, name));

        std::stringstream sstr;
        sstr << std::setfill ('0') << std::setw(2) << std::hex << id;
        std::string idHex = sstr.str();

        std::string uri = "radio://" + std::to_string(m_radio) + "/" + std::to_string(channel) + "/2M/E7E7E7E7" + idHex;
        std::string tf_prefix = "cf" + std::to_string(id);
        std::string frame = "cf" + std::to_string(id);
        cfConfigs.push_back({uri, tf_prefix, frame, id, type});
      }
    }
    ROS_INFO("Parsed crazyflies.yaml successfully.");

    // add Crazyflies
    for (const auto& config : cfConfigs) {
      addCrazyflie(config.uri, config.tf_prefix, config.frame, "/world", config.idNumber, config.type, logBlocks);

      auto start = std::chrono::high_resolution_clock::now();
      updateParams(m_cfs.back());
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      ROS_INFO("Update params: %f s", elapsed.count());
    }
  }

  void addCrazyflie(
    const std::string& uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    int id,
    const std::string& type,
    const std::vector<crazyswarm::LogBlock>& logBlocks)
  {
    ROS_INFO("Adding CF: %s (%s, %s)...", tf_prefix.c_str(), uri.c_str(), frame.c_str());
    auto start = std::chrono::high_resolution_clock::now();
    CrazyflieROS* cf = new CrazyflieROS(
      uri,
      tf_prefix,
      frame,
      worldFrame,
      id,
      type,
      logBlocks,
      m_slowQueue);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("CF ctor: %f s", elapsed.count());
    cf->run(m_slowQueue);
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = end2 - end;
    ROS_INFO("CF run: %f s", elapsed2.count());
    m_cfs.push_back(cf);
  }

  void updateParams(
    CrazyflieROS* cf)
  {
    ros::NodeHandle n("~");
    ros::NodeHandle nGlobal;
    // update parameters
    // std::cout << "attempt: " << "firmwareParams/" + cf->type() << std::endl;
    // char dummy;
    // std::cin >> dummy;

    // update global, type-specific, and CF-specific parameters
    std::vector<XmlRpc::XmlRpcValue> firmwareParamsVec(2);
    n.getParam("firmwareParams", firmwareParamsVec[0]);
    nGlobal.getParam("crazyflieTypes/" + cf->type() + "/firmwareParams", firmwareParamsVec[1]);

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      if (id == cf->id()) {
        if (crazyflie.hasMember("firmwareParams")) {
          firmwareParamsVec.push_back(crazyflie["firmwareParams"]);
        }
        break;
      }
    }


    crazyswarm::UpdateParams::Request request;
    crazyswarm::UpdateParams::Response response;

    for (auto& firmwareParams : firmwareParamsVec) {
      // ROS_ASSERT(firmwareParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
      auto iter = firmwareParams.begin();
      for (; iter != firmwareParams.end(); ++iter) {
        std::string group = iter->first;
        XmlRpc::XmlRpcValue v = iter->second;
        auto iter2 = v.begin();
        for (; iter2 != v.end(); ++iter2) {
          std::string param = iter2->first;
          XmlRpc::XmlRpcValue value = iter2->second;
          if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            bool b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            double b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            // "1e-5" is not recognize as double; convert manually here
            std::string value_str = value;
            double value = std::stod(value_str);
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, value);
            std::cout << "update " << group + "/" + param << " to " << value << std::endl;
          } else {
            ROS_ERROR("No known type for %s.%s! (type: %d)", group.c_str(), param.c_str(), value.getType());
          }
          request.params.push_back(group + "/" + param);

        }
      }
    }
    cf->updateParams(request, response);
  }

private:
  std::vector<CrazyflieROS*> m_cfs;
  std::string m_interactiveObject;
  libobjecttracker::ObjectTracker* m_tracker; // non-owning pointer
  int m_radio;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pMarkers;
  std::map<std::string, libmotioncapture::RigidBody>* m_pMocapRigidBodies; // non-owning pointer
  ros::CallbackQueue m_slowQueue;
  CrazyflieBroadcaster m_cfbc;
  bool m_isEmergency;
  bool m_useMotionCaptureObjectTracking;
  tf::TransformBroadcaster m_br;
  latency m_latency;
  bool m_sendPositionOnly;
  std::vector<std::unique_ptr<std::ofstream>> m_outputCSVs;
  int m_phase;
  std::chrono::high_resolution_clock::time_point m_phaseStart;
};

// handles all Crazyflies
class CrazyflieServer
{
public:
  CrazyflieServer()
    : m_isEmergency(false)
    , m_serviceEmergency()
    , m_serviceStartTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceGoTo()
    , m_lastInteractiveObjectPosition(-10, -10, 1)
    , m_broadcastingNumRepeats(15)
    , m_broadcastingDelayBetweenRepeatsMs(1)
  {
    ros::NodeHandle nh;
    nh.setCallbackQueue(&m_queue);

    m_serviceEmergency = nh.advertiseService("emergency", &CrazyflieServer::emergency, this);
    m_serviceStartTrajectory = nh.advertiseService("start_trajectory", &CrazyflieServer::startTrajectory, this);
    m_serviceTakeoff = nh.advertiseService("takeoff", &CrazyflieServer::takeoff, this);
    m_serviceLand = nh.advertiseService("land", &CrazyflieServer::land, this);
    m_serviceGoTo = nh.advertiseService("go_to", &CrazyflieServer::goTo, this);

    m_serviceUpdateParams = nh.advertiseService("update_params", &CrazyflieServer::updateParams, this);

    m_pubPointCloud = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 1);

    m_subscribeVirtualInteractiveObject = nh.subscribe("virtual_interactive_object", 1, &CrazyflieServer::virtualInteractiveObjectCallback, this);
  }

  ~CrazyflieServer()
  {
    for (CrazyflieGroup* group : m_groups) {
      delete group;
    }
  }

  void virtualInteractiveObjectCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    m_lastInteractiveObjectPosition = Eigen::Vector3f(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  }

  void run()
  {
    std::thread tSlow(&CrazyflieServer::runSlow, this);
    runFast();
    tSlow.join();
  }

  void runFast()
  {
    std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;
    std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
    std::set<int> channels;

    readMarkerConfigurations(markerConfigurations);
    readDynamicsConfigurations(dynamicsConfigurations);
    readChannels(channels);

    bool useMotionCaptureObjectTracking;
    std::string logFilePath;
    std::string interactiveObject;
    bool printLatency;
    bool writeCSVs;
    bool sendPositionOnly;
    std::string motionCaptureType;

    ros::NodeHandle nl("~");
    std::string objectTrackingType;
    nl.getParam("object_tracking_type", objectTrackingType);
    useMotionCaptureObjectTracking = (objectTrackingType == "motionCapture");
    nl.param<std::string>("save_point_clouds", logFilePath, "");
    nl.param<std::string>("interactive_object", interactiveObject, "");
    nl.getParam("print_latency", printLatency);
    nl.getParam("write_csvs", writeCSVs);
    nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

    nl.param<int>("broadcasting_num_repeats", m_broadcastingNumRepeats, 15);
    nl.param<int>("broadcasting_delay_between_repeats_ms", m_broadcastingDelayBetweenRepeatsMs, 1);
    nl.param<bool>("send_position_only", sendPositionOnly, false);

    // tilde-expansion
    wordexp_t wordexp_result;
    if (wordexp(logFilePath.c_str(), &wordexp_result, 0) == 0) {
      // success - only read first result, could be more if globs were used
      logFilePath = wordexp_result.we_wordv[0];
    }
    wordfree(&wordexp_result);

    libobjecttracker::PointCloudLogger pointCloudLogger(logFilePath);
    const bool logClouds = !logFilePath.empty();

    // custom log blocks
    std::vector<std::string> genericLogTopics;
    nl.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
    std::vector<int> genericLogTopicFrequencies;
    nl.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());
    std::vector<crazyswarm::LogBlock> logBlocks;
    if (genericLogTopics.size() == genericLogTopicFrequencies.size())
    {
      size_t i = 0;
      for (auto& topic : genericLogTopics)
      {
        crazyswarm::LogBlock logBlock;
        logBlock.topic_name = topic;
        logBlock.frequency = genericLogTopicFrequencies[i];
        nl.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
        logBlocks.push_back(logBlock);
        ++i;
      }
    }
    else
    {
      ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
    }

    // Make a new client
    std::map<std::string, std::string> cfg;
    std::string hostname;
    nl.getParam("motion_capture_host_name", hostname);
    cfg["hostname"] = hostname;
    if (nl.hasParam("motion_capture_interface_ip")) {
      std::string interface_ip;
      nl.param<std::string>("motion_capture_interface_ip", interface_ip);
      cfg["interface_ip"] = interface_ip;
    }

    std::unique_ptr<libmotioncapture::MotionCapture> mocap;

    if (motionCaptureType != "none") {
      ROS_INFO(
        "libmotioncapture connecting to %s at hostname '%s' - "
        "might block indefinitely if unreachable!",
        motionCaptureType.c_str(),
        hostname.c_str()
      );
      mocap.reset(libmotioncapture::MotionCapture::connect(motionCaptureType, cfg));
      if (!mocap) {
        throw std::runtime_error("Unknown motion capture type!");
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
    std::map<std::string, libmotioncapture::RigidBody> mocapRigidBodies;

    // Create all groups in parallel and launch threads
    {
      std::vector<std::future<CrazyflieGroup*> > handles;
      int r = 0;
      std::cout << "ch: " << channels.size() << std::endl;
      for (int channel : channels) {
        auto handle = std::async(std::launch::async,
            [&](int channel, int radio)
            {
              // std::cout << "radio: " << radio << std::endl;
              return new CrazyflieGroup(
                dynamicsConfigurations,
                markerConfigurations,
                // &client,
                markers,
                &mocapRigidBodies,
                radio,
                channel,
                useMotionCaptureObjectTracking,
                logBlocks,
                interactiveObject,
                writeCSVs,
                sendPositionOnly);
            },
            channel,
            r
          );
        handles.push_back(std::move(handle));
        ++r;
      }

      for (auto& handle : handles) {
        m_groups.push_back(handle.get());
      }
    }

    // start the groups threads
    std::vector<std::thread> threads;
    for (auto& group : m_groups) {
      threads.push_back(std::thread(&CrazyflieGroup::runSlow, group));
    }

    ROS_INFO("Started %lu threads", threads.size());

    // Connect to a server
    // ROS_INFO("Connecting to %s ...", hostName.c_str());
    // while (ros::ok() && !client.IsConnected().Connected) {
    //   // Direct connection
    //   bool ok = (client.Connect(hostName).Result == Result::Success);
    //   if(!ok) {
    //     ROS_WARN("Connect failed...");
    //   }
    //   ros::spinOnce();
    // }
    if (mocap) {

      // setup messages
      sensor_msgs::PointCloud msgPointCloud;
      msgPointCloud.header.seq = 0;
      msgPointCloud.header.frame_id = "world";

      auto startTime = std::chrono::high_resolution_clock::now();

      struct latencyEntry {
        std::string name;
        double secs;
      };
      std::vector<latencyEntry> latencies;

      std::vector<double> latencyTotal(6 + 3 * 2, 0.0);
      uint32_t latencyCount = 0;

      while (ros::ok() && !m_isEmergency) {
        // Get a frame
        mocap->waitForNextFrame();

        latencies.clear();

        auto startIteration = std::chrono::high_resolution_clock::now();
        double totalLatency = 0;

        // Get the latency
        const auto& mocapLatency = mocap->latency();
        float totalMocapLatency = 0;
        for (const auto& item : mocapLatency) {
          totalMocapLatency += item.value();
        }
        if (totalMocapLatency > 0.035) {
          std::stringstream sstr;
          sstr << "MoCap Latency high: " << totalMocapLatency << " s." << std::endl;
          for (const auto& item : mocapLatency) {
            sstr << "  Latency: " << item.name() << ": " << item.value() << " s." << std::endl;
          }
          ROS_WARN("%s", sstr.str().c_str());
        }

        if (printLatency) {
          size_t i = 0;
          for (const auto& item : mocapLatency) {
            latencies.push_back({item.name(), item.value()});
            latencyTotal[i] += item.value();
            totalLatency += item.value();
            latencyTotal.back() += item.value();
          }
          ++i;
        }

        // size_t latencyCount = client.GetLatencySampleCount().Count;
        // for(size_t i = 0; i < latencyCount; ++i) {
        //   std::string sampleName  = client.GetLatencySampleName(i).Name;
        //   double      sampleValue = client.GetLatencySampleValue(sampleName).Value;

        //   ROS_INFO("Latency: %s: %f", sampleName.c_str(), sampleValue);
        // }

        // Get the unlabeled markers and create point cloud
        if (!useMotionCaptureObjectTracking) {
          // ToDO: If we switch our datastructure to pointcloud2 (here, for the ROS publisher, and libobjecttracker)
          //       we can avoid a copy here.
          const auto& pointcloud = mocap->pointCloud();
          markers->clear();
          for (size_t i = 0; i < pointcloud.rows(); ++i) {
            const auto &point = pointcloud.row(i);
            markers->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
          }

          msgPointCloud.header.seq += 1;
          msgPointCloud.header.stamp = ros::Time::now();
          msgPointCloud.points.resize(markers->size());
          for (size_t i = 0; i < markers->size(); ++i) {
            const pcl::PointXYZ& point = markers->at(i);
            msgPointCloud.points[i].x = point.x;
            msgPointCloud.points[i].y = point.y;
            msgPointCloud.points[i].z = point.z;
          }
          m_pubPointCloud.publish(msgPointCloud);

          if (logClouds) {
            pointCloudLogger.log(markers);
          }
        }

        if (useMotionCaptureObjectTracking || !interactiveObject.empty()) {
          // get mocap rigid bodies
          mocapRigidBodies = mocap->rigidBodies();
          if (interactiveObject == "virtual") {
            Eigen::Quaternionf quat(0, 0, 0, 1);
            mocapRigidBodies.emplace(interactiveObject,
                libmotioncapture::RigidBody(
                    interactiveObject,
                    m_lastInteractiveObjectPosition,
                    quat));
          }
        }

        auto startRunGroups = std::chrono::high_resolution_clock::now();
        std::vector<std::future<void> > handles;
        for (auto group : m_groups) {
          auto handle = std::async(std::launch::async, &CrazyflieGroup::runFast, group);
          handles.push_back(std::move(handle));
        }

        for (auto& handle : handles) {
          handle.wait();
        }
        auto endRunGroups = std::chrono::high_resolution_clock::now();
        if (printLatency) {
          std::chrono::duration<double> elapsedRunGroups = endRunGroups - startRunGroups;
          latencies.push_back({"Run All Groups", elapsedRunGroups.count()});
          latencyTotal[4] += elapsedRunGroups.count();
          totalLatency += elapsedRunGroups.count();
          latencyTotal.back() += elapsedRunGroups.count();
          int groupId = 0;
          for (auto group : m_groups) {
            auto latency = group->lastLatency();
            int radio = group->radio();
            latencies.push_back({"Group " + std::to_string(radio) + " objectTracking", latency.objectTracking});
            latencies.push_back({"Group " + std::to_string(radio) + " broadcasting", latency.broadcasting});
            latencyTotal[5 + 2*groupId] += latency.objectTracking;
            latencyTotal[6 + 2*groupId] += latency.broadcasting;
            ++groupId;
          }
        }

        auto endIteration = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = endIteration - startIteration;
        double elapsedSeconds = elapsed.count();
        if (elapsedSeconds > 0.009) {
          ROS_WARN("Latency too high! Is %f s.", elapsedSeconds);
        }

        if (printLatency) {
          ++latencyCount;
          std::cout << "Latencies" << std::endl;
          for (auto& latency : latencies) {
            std::cout << latency.name << ": " << latency.secs * 1000 << " ms" << std::endl;
          }
          std::cout << "Total " << totalLatency * 1000 << " ms" << std::endl;
          // // if (latencyCount % 100 == 0) {
            std::cout << "Avg " << latencyCount << std::endl;
            for (size_t i = 0; i < latencyTotal.size(); ++i) {
              std::cout << latencyTotal[i] / latencyCount * 1000.0 << ",";
            }
            std::cout << std::endl;
          // // }
        }

        // ROS_INFO("Latency: %f s", elapsedSeconds.count());

        // m_fastQueue.callAvailable(ros::WallDuration(0));
      }

      if (logClouds) {
        pointCloudLogger.flush();
      }
    }

    // wait for other threads
    for (auto& thread : threads) {
      thread.join();
    }
  }

  void runSlow()
  {
    while(ros::ok() && !m_isEmergency) {
      m_queue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->emergency();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
    m_isEmergency = true;

    return true;
  }

  bool takeoff(
    crazyswarm::Takeoff::Request& req,
    crazyswarm::Takeoff::Response& res)
  {
    ROS_INFO("Takeoff!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->takeoff(req.height, req.duration.toSec(), req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool land(
    crazyswarm::Land::Request& req,
    crazyswarm::Land::Response& res)
  {
    ROS_INFO("Land!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->land(req.height, req.duration.toSec(), req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool goTo(
    crazyswarm::GoTo::Request& req,
    crazyswarm::GoTo::Response& res)
  {
    ROS_INFO("GoTo!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool startTrajectory(
    crazyswarm::StartTrajectory::Request& req,
    crazyswarm::StartTrajectory::Response& res)
  {
    ROS_INFO("Start trajectory!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool updateParams(
    crazyswarm::UpdateParams::Request& req,
    crazyswarm::UpdateParams::Response& res)
  {
    ROS_INFO("UpdateParams!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->updateParams(req.params);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

//
  void readMarkerConfigurations(
    std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations)
  {
    markerConfigurations.clear();
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numMarkerConfigurations", numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
      std::stringstream sstr;
      sstr << "markerConfigurations/" << i << "/numPoints";
      int numPoints;
      nGlobal.getParam(sstr.str(), numPoints);

      std::vector<double> offset;
      std::stringstream sstr2;
      sstr2 << "markerConfigurations/" << i << "/offset";
      nGlobal.getParam(sstr2.str(), offset);
      for (int j = 0; j < numPoints; ++j) {
        std::stringstream sstr3;
        sstr3 << "markerConfigurations/" << i << "/points/" << j;
        std::vector<double> points;
        nGlobal.getParam(sstr3.str(), points);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
  }

  void readDynamicsConfigurations(
    std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations)
  {
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numDynamicsConfigurations", numConfigurations);
    dynamicsConfigurations.resize(numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      std::stringstream sstr;
      sstr << "dynamicsConfigurations/" << i;
      nGlobal.getParam(sstr.str() + "/maxXVelocity", dynamicsConfigurations[i].maxXVelocity);
      nGlobal.getParam(sstr.str() + "/maxYVelocity", dynamicsConfigurations[i].maxYVelocity);
      nGlobal.getParam(sstr.str() + "/maxZVelocity", dynamicsConfigurations[i].maxZVelocity);
      nGlobal.getParam(sstr.str() + "/maxPitchRate", dynamicsConfigurations[i].maxPitchRate);
      nGlobal.getParam(sstr.str() + "/maxRollRate", dynamicsConfigurations[i].maxRollRate);
      nGlobal.getParam(sstr.str() + "/maxYawRate", dynamicsConfigurations[i].maxYawRate);
      nGlobal.getParam(sstr.str() + "/maxRoll", dynamicsConfigurations[i].maxRoll);
      nGlobal.getParam(sstr.str() + "/maxPitch", dynamicsConfigurations[i].maxPitch);
      nGlobal.getParam(sstr.str() + "/maxFitnessScore", dynamicsConfigurations[i].maxFitnessScore);
    }
  }

  void readChannels(
    std::set<int>& channels)
  {
    // read CF config
    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    channels.clear();
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int channel = crazyflie["channel"];
      channels.insert(channel);
    }
  }

private:
  std::string m_worldFrame;
  bool m_isEmergency;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceStartTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceGoTo;
  ros::ServiceServer m_serviceUpdateParams;

  ros::Publisher m_pubPointCloud;
  // tf::TransformBroadcaster m_br;

  std::vector<CrazyflieGroup*> m_groups;

  ros::Subscriber m_subscribeVirtualInteractiveObject;
  Eigen::Vector3f m_lastInteractiveObjectPosition;

  int m_broadcastingNumRepeats;
  int m_broadcastingDelayBetweenRepeatsMs;

private:
  // We have two callback queues
  // 1. Fast queue handles pose and emergency callbacks. Those are high-priority and can be served quickly
  // 2. Slow queue handles all other requests.
  // Each queue is handled in its own thread. We don't want a thread per CF to make sure that the fast queue
  //  gets called frequently enough.

  ros::CallbackQueue m_queue;
  // ros::CallbackQueue m_slowQueue;
};

int main(int argc, char **argv)
{
  // raise(SIGSTOP);

  ros::init(argc, argv, "crazyflie_server");

  // ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("world_frame", worldFrame, "/world");
  // std::string broadcastUri;
  // n.getParam("broadcast_uri", broadcastUri);

  CrazyflieServer server;//(broadcastUri, worldFrame);

  // read CF config
  ros::NodeHandle nGlobal;

  XmlRpc::XmlRpcValue crazyflies;
  nGlobal.getParam("crazyflies", crazyflies);
  ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::set<int> cfIds;
  for (int32_t i = 0; i < crazyflies.size(); ++i)
  {
    ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
    int id = crazyflie["id"];
    int channel = crazyflie["channel"];
    if (cfIds.find(id) != cfIds.end()) {
      ROS_FATAL("CF with the same id twice in configuration!");
      return 1;
    }
    cfIds.insert(id);
  }

  // ROS_INFO("All CFs are ready!");

  server.run();

  return 0;
}


//Codigo original

// class ROSLogger : public Logger
// {
// public:
//   ROSLogger()
//     : Logger()
//   {
//   }

//   virtual ~ROSLogger() {}

//   virtual void info(const std::string& msg)
//   {
//     ROS_INFO("%s", msg.c_str());
//   }

//   virtual void warning(const std::string& msg)
//   {
//     ROS_WARN("%s", msg.c_str());
//   }

//   virtual void error(const std::string& msg)
//   {
//     ROS_ERROR("%s", msg.c_str());
//   }
// };

// static ROSLogger rosLogger;

// class CrazyflieROS
// {
// public:
//   CrazyflieROS(
//     const std::string& link_uri,
//     const std::string& tf_prefix,
//     float roll_trim,
//     float pitch_trim,
//     bool enable_logging,
//     bool enable_parameters,
//     std::vector<crazyflie_driver::LogBlock>& log_blocks,
//     bool use_ros_time,
//     bool enable_logging_imu,
//     bool enable_logging_temperature,
//     bool enable_logging_magnetic_field,
//     bool enable_logging_pressure,
//     bool enable_logging_battery,
//     bool enable_logging_pose,
//     bool enable_logging_packets)
//     : m_tf_prefix(tf_prefix)
//     , m_cf(
//       link_uri,
//       rosLogger,
//       std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1))
//     , m_isEmergency(false)
//     , m_roll_trim(roll_trim)
//     , m_pitch_trim(pitch_trim)
//     , m_enableLogging(enable_logging)
//     , m_enableParameters(enable_parameters)
//     , m_logBlocks(log_blocks)
//     , m_use_ros_time(use_ros_time)
//     , m_enable_logging_imu(enable_logging_imu)
//     , m_enable_logging_temperature(enable_logging_temperature)
//     , m_enable_logging_magnetic_field(enable_logging_magnetic_field)
//     , m_enable_logging_pressure(enable_logging_pressure)
//     , m_enable_logging_battery(enable_logging_battery)
//     , m_enable_logging_pose(enable_logging_pose)
//     , m_enable_logging_packets(enable_logging_packets)
//     , m_serviceEmergency()
//     , m_serviceUpdateParams()
//     , m_serviceSetGroupMask()
//     , m_serviceTakeoff()
//     , m_serviceLand()
//     , m_serviceStop()
//     , m_serviceGoTo()
//     , m_serviceUploadTrajectory()
//     , m_serviceStartTrajectory()
//     , m_serviceNotifySetpointsStop()
//     , m_subscribeCmdVel()
//     , m_subscribeCmdFullState()
//     , m_subscribeCmdVelocityWorld()
//     , m_subscribeCmdHover()
//     , m_subscribeCmdStop()
//     , m_subscribeCmdPosition()
//     , m_subscribeExternalPosition()
//     , m_pubImu()
//     , m_pubTemp()
//     , m_pubMag()
//     , m_pubPressure()
//     , m_pubBattery()
//     , m_pubRssi()
//     , m_sentSetpoint(false)
//     , m_sentExternalPosition(false)
//   {
//     m_thread = std::thread(&CrazyflieROS::run, this);
//   }

//   void stop()
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "Disconnecting ...");
//     m_isEmergency = true;
//     m_thread.join();
//   }

//   /**
//    * Service callback which transmits a packet to the crazyflie
//    * @param  req The service request, which contains a crtpPacket to transmit.
//    * @param  res The service response, which is not used.
//    * @return     returns true always
//    */
//   bool sendPacket (
//     crazyflie_driver::sendPacket::Request &req,
//     crazyflie_driver::sendPacket::Response &res)
//   {
//     /** Convert the message struct to the packet struct */
//     crtpPacket_t packet;
//     packet.size = req.packet.size;
//     packet.header = req.packet.header;
//     for (int i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
//       packet.data[i] = req.packet.data[i];
//     }
//     m_cf.queueOutgoingPacket(packet);
//     return true;
//   }

// private:
//   struct logImu {
//     float acc_x;
//     float acc_y;
//     float acc_z;
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;
//   } __attribute__((packed));

//   struct log2 {
//     float mag_x;
//     float mag_y;
//     float mag_z;
//     float baro_temp;
//     float baro_pressure;
//     float pm_vbat;
//   } __attribute__((packed));

//   struct logPose {
//     float x;
//     float y;
//     float z;
//     int32_t quatCompressed;
//   } __attribute__((packed));

// private:
//   bool emergency(
//     std_srvs::Empty::Request& req,
//     std_srvs::Empty::Response& res)
//   {
//     ROS_FATAL_NAMED(m_tf_prefix, "Emergency requested!");
//     m_isEmergency = true;
//     m_cf.emergencyStop();

//     return true;
//   }

//   template<class T, class U>
//   void updateParam(uint16_t id, const std::string& ros_param) {
//       U value;
//       ros::param::get(ros_param, value);
//       m_cf.setParam<T>(id, (T)value);
//   }

// void cmdHoverSetpoint(
//     const crazyflie_driver::Hover::ConstPtr& msg)
//   {
//      //ROS_INFO("got a hover setpoint");
//     if (!m_isEmergency) {
//       float vx = msg->vx;
//       float vy = msg->vy;
//       float yawRate = msg->yawrate;
//       float zDistance = msg->zDistance;

//       m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
//       m_sentSetpoint = true;
//       //ROS_INFO("set a hover setpoint");
//     }
//   }

// void cmdStop(
//     const std_msgs::Empty::ConstPtr& msg)
//   {
//      //ROS_INFO("got a stop setpoint");
//     if (!m_isEmergency) {
//       m_cf.sendStop();
//       m_sentSetpoint = true;
//       //ROS_INFO("set a stop setpoint");
//     }
//   }

// void cmdPositionSetpoint(
//     const crazyflie_driver::Position::ConstPtr& msg)
//   {
//     if(!m_isEmergency) {
//       float x = msg->x;
//       float y = msg->y;
//       float z = msg->z;
//       float yaw = msg->yaw;

//       m_cf.sendPositionSetpoint(x, y, z, yaw);
//       m_sentSetpoint = true;
//     }
//   }

//   bool updateParams(
//     crazyflie_driver::UpdateParams::Request& req,
//     crazyflie_driver::UpdateParams::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "Update parameters");
//     for (auto&& p : req.params) {
//       std::string ros_param = "/" + m_tf_prefix + "/" + p;
//       size_t pos = p.find("/");
//       std::string group(p.begin(), p.begin() + pos);
//       std::string name(p.begin() + pos + 1, p.end());

//       auto entry = m_cf.getParamTocEntry(group, name);
//       if (entry)
//       {
//         switch (entry->type) {
//           case Crazyflie::ParamTypeUint8:
//             updateParam<uint8_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeInt8:
//             updateParam<int8_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeUint16:
//             updateParam<uint16_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeInt16:
//             updateParam<int16_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeUint32:
//             updateParam<uint32_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeInt32:
//             updateParam<int32_t, int>(entry->id, ros_param);
//             break;
//           case Crazyflie::ParamTypeFloat:
//             updateParam<float, float>(entry->id, ros_param);
//             break;
//         }
//       }
//       else {
//         ROS_ERROR_NAMED(m_tf_prefix, "Could not find param %s/%s", group.c_str(), name.c_str());
//       }
//     }
//     return true;
//   }

//   void cmdVelChanged(
//     const geometry_msgs::Twist::ConstPtr& msg)
//   {
//     if (!m_isEmergency) {
//       float roll = msg->linear.y + m_roll_trim;
//       float pitch = - (msg->linear.x + m_pitch_trim);
//       float yawrate = msg->angular.z;
//       uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

//       m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
//       m_sentSetpoint = true;
//     }
//   }

//   void cmdFullStateSetpoint(
//     const crazyflie_driver::FullState::ConstPtr& msg)
//   {
//     //ROS_INFO("got a full state setpoint");
//     if (!m_isEmergency) {
//       float x = msg->pose.position.x;
//       float y = msg->pose.position.y;
//       float z = msg->pose.position.z;
//       float vx = msg->twist.linear.x;
//       float vy = msg->twist.linear.y;
//       float vz = msg->twist.linear.z;
//       float ax = msg->acc.x;
//       float ay = msg->acc.y;
//       float az = msg->acc.z;

//       float qx = msg->pose.orientation.x;
//       float qy = msg->pose.orientation.y;
//       float qz = msg->pose.orientation.z;
//       float qw = msg->pose.orientation.w;
//       float rollRate = msg->twist.angular.x;
//       float pitchRate = msg->twist.angular.y;
//       float yawRate = msg->twist.angular.z;

//       m_cf.sendFullStateSetpoint(
//         x, y, z,
//         vx, vy, vz,
//         ax, ay, az,
//         qx, qy, qz, qw,
//         rollRate, pitchRate, yawRate);
//       m_sentSetpoint = true;
//       //ROS_INFO("set a full state setpoint");
//     }
//   }

//   void cmdVelocityWorldSetpoint(
//     const crazyflie_driver::VelocityWorld::ConstPtr& msg)
//   {
//     //ROS_INFO("got a velocity world setpoint");
//     if (!m_isEmergency) {
//       float x = msg->vel.x;
//       float y = msg->vel.y;
//       float z = msg->vel.z;
//       float yawRate = msg->yawRate;

//       m_cf.sendVelocityWorldSetpoint(
//         x, y, z, yawRate);
//       m_sentSetpoint = true;
//       //ROS_INFO("set a velocity world setpoint");
//     }
//   }

//   void positionMeasurementChanged(
//     const geometry_msgs::PointStamped::ConstPtr& msg)
//   {
//     m_cf.sendExternalPositionUpdate(msg->point.x, msg->point.y, msg->point.z);
//     m_sentExternalPosition = true;
//   }

//   void poseMeasurementChanged(
//     const geometry_msgs::PoseStamped::ConstPtr& msg)
//   {
//     m_cf.sendExternalPoseUpdate(
//       msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
//       msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
//     m_sentExternalPosition = true;
//   }

//   void run()
//   {
//     ros::NodeHandle n;
//     n.setCallbackQueue(&m_callback_queue);

//     m_subscribeCmdVel = n.subscribe(m_tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
//     m_subscribeCmdFullState = n.subscribe(m_tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
//     m_subscribeCmdVelocityWorld = n.subscribe(m_tf_prefix+"/cmd_velocity_world", 1, &CrazyflieROS::cmdVelocityWorldSetpoint, this);
//     m_subscribeExternalPosition = n.subscribe(m_tf_prefix + "/external_position", 1, &CrazyflieROS::positionMeasurementChanged, this);
//     m_subscribeExternalPose = n.subscribe(m_tf_prefix + "/external_pose", 1, &CrazyflieROS::poseMeasurementChanged, this);
//     m_serviceEmergency = n.advertiseService(m_tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
//     m_subscribeCmdHover = n.subscribe(m_tf_prefix + "/cmd_hover", 1, &CrazyflieROS::cmdHoverSetpoint, this);
//     m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
//     m_subscribeCmdPosition = n.subscribe(m_tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);


//     m_serviceSetGroupMask = n.advertiseService(m_tf_prefix + "/set_group_mask", &CrazyflieROS::setGroupMask, this);
//     m_serviceTakeoff = n.advertiseService(m_tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
//     m_serviceLand = n.advertiseService(m_tf_prefix + "/land", &CrazyflieROS::land, this);
//     m_serviceStop = n.advertiseService(m_tf_prefix + "/stop", &CrazyflieROS::stop, this);
//     m_serviceGoTo = n.advertiseService(m_tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
//     m_serviceUploadTrajectory = n.advertiseService(m_tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);
//     m_serviceStartTrajectory = n.advertiseService(m_tf_prefix + "/start_trajectory", &CrazyflieROS::startTrajectory, this);
//     m_serviceNotifySetpointsStop = n.advertiseService(m_tf_prefix + "/notify_setpoints_stop", &CrazyflieROS::notifySetpointsStop, this);

//     if (m_enable_logging_imu) {
//       m_pubImu = n.advertise<sensor_msgs::Imu>(m_tf_prefix + "/imu", 10);
//     }
//     if (m_enable_logging_temperature) {
//       m_pubTemp = n.advertise<sensor_msgs::Temperature>(m_tf_prefix + "/temperature", 10);
//     }
//     if (m_enable_logging_magnetic_field) {
//       m_pubMag = n.advertise<sensor_msgs::MagneticField>(m_tf_prefix + "/magnetic_field", 10);
//     }
//     if (m_enable_logging_pressure) {
//       m_pubPressure = n.advertise<std_msgs::Float32>(m_tf_prefix + "/pressure", 10);
//     }
//     if (m_enable_logging_battery) {
//       m_pubBattery = n.advertise<std_msgs::Float32>(m_tf_prefix + "/battery", 10);
//     }
//     if (m_enable_logging_pose) {
//       m_pubPose = n.advertise<geometry_msgs::PoseStamped>(m_tf_prefix + "/pose", 10);
//     }
//     if (m_enable_logging_packets) {
//       m_pubPackets = n.advertise<crazyflie_driver::crtpPacket>(m_tf_prefix + "/packets", 10);
//       std::function<void(const ITransport::Ack&)> cb_genericPacket = std::bind(&CrazyflieROS::onGenericPacket, this, std::placeholders::_1);
//       m_cf.setGenericPacketCallback(cb_genericPacket);
//     }
//     m_pubRssi = n.advertise<std_msgs::Float32>(m_tf_prefix + "/rssi", 10);

//     for (auto& logBlock : m_logBlocks)
//     {
//       m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(m_tf_prefix + "/" + logBlock.topic_name, 10));
//     }

//     m_sendPacketServer = n.advertiseService(m_tf_prefix + "/send_packet"  , &CrazyflieROS::sendPacket, this);

//     // m_cf.reboot();

//     auto start = std::chrono::system_clock::now();

//     m_cf.logReset();

//     std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
//     m_cf.setLinkQualityCallback(cb_lq);

//     if (m_enableParameters)
//     {
//       ROS_INFO_NAMED(m_tf_prefix, "Requesting parameters...");
//       m_cf.requestParamToc();
//       for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
//         auto entry = *iter;
//         std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
//         switch (entry.type) {
//           case Crazyflie::ParamTypeUint8:
//             ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeInt8:
//             ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeUint16:
//             ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeInt16:
//             ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeUint32:
//             ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeInt32:
//             ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
//             break;
//           case Crazyflie::ParamTypeFloat:
//             ros::param::set(paramName, m_cf.getParam<float>(entry.id));
//             break;
//         }
//       }
//       m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
//     }

//     std::unique_ptr<LogBlock<logImu> > logBlockImu;
//     std::unique_ptr<LogBlock<log2> > logBlock2;
//     std::unique_ptr<LogBlock<logPose> > logBlockPose;
//     std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
//     if (m_enableLogging) {

//       std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
//       m_cf.setEmptyAckCallback(cb_ack);

//       ROS_INFO_NAMED(m_tf_prefix, "Requesting Logging variables...");
//       m_cf.requestLogToc();

//       if (m_enable_logging_imu) {
//         std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

//         logBlockImu.reset(new LogBlock<logImu>(
//           &m_cf,{
//             {"acc", "x"},
//             {"acc", "y"},
//             {"acc", "z"},
//             {"gyro", "x"},
//             {"gyro", "y"},
//             {"gyro", "z"},
//           }, cb));
//         logBlockImu->start(1); // 10ms
//       }

//       if (   m_enable_logging_temperature
//           || m_enable_logging_magnetic_field
//           || m_enable_logging_pressure
//           || m_enable_logging_battery)
//       {
//         std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

//         logBlock2.reset(new LogBlock<log2>(
//           &m_cf,{
//             {"mag", "x"},
//             {"mag", "y"},
//             {"mag", "z"},
//             {"baro", "temp"},
//             {"baro", "pressure"},
//             {"pm", "vbat"},
//           }, cb2));
//         logBlock2->start(10); // 100ms
//       }

//       if (m_enable_logging_pose) {
//         std::function<void(uint32_t, logPose*)> cb = std::bind(&CrazyflieROS::onPoseData, this, std::placeholders::_1, std::placeholders::_2);

//         logBlockPose.reset(new LogBlock<logPose>(
//           &m_cf,{
//             {"stateEstimate", "x"},
//             {"stateEstimate", "y"},
//             {"stateEstimate", "z"},
//             {"stateEstimateZ", "quat"}
//           }, cb));
//         logBlockPose->start(1); // 10ms
//       }

//       // custom log blocks
//       size_t i = 0;
//       for (auto& logBlock : m_logBlocks)
//       {
//         std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
//           std::bind(
//             &CrazyflieROS::onLogCustom,
//             this,
//             std::placeholders::_1,
//             std::placeholders::_2,
//             std::placeholders::_3);

//         logBlocksGeneric[i].reset(new LogBlockGeneric(
//           &m_cf,
//           logBlock.variables,
//           (void*)&m_pubLogDataGeneric[i],
//           cb));
//         logBlocksGeneric[i]->start(logBlock.frequency / 10);
//         ++i;
//       }


//     }

//     ROS_INFO_NAMED(m_tf_prefix, "Requesting memories...");
//     m_cf.requestMemoryToc();

//     ROS_INFO_NAMED(m_tf_prefix, "Ready...");
//     auto end = std::chrono::system_clock::now();
//     std::chrono::duration<double> elapsedSeconds = end-start;
//     ROS_INFO_NAMED(m_tf_prefix, "Elapsed: %f s", elapsedSeconds.count());

//     // Send 0 thrust initially for thrust-lock
//     for (int i = 0; i < 100; ++i) {
//        m_cf.sendSetpoint(0, 0, 0, 0);
//     }

//     while(!m_isEmergency) {
//       // make sure we ping often enough to stream data out
//       if (m_enableLogging && !m_sentSetpoint && !m_sentExternalPosition) {
//         m_cf.transmitPackets();
//         m_cf.sendPing();
//       }
//       m_sentSetpoint = false;
//       m_sentExternalPosition = false;

//       // Execute any ROS related functions now
//       m_callback_queue.callAvailable(ros::WallDuration(0.0));
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }

//     // Make sure we turn the engines off
//     for (int i = 0; i < 100; ++i) {
//        m_cf.sendSetpoint(0, 0, 0, 0);
//     }

//   }

//   void onImuData(uint32_t time_in_ms, logImu* data) {
//     if (m_enable_logging_imu) {
//       sensor_msgs::Imu msg;
//       if (m_use_ros_time) {
//         msg.header.stamp = ros::Time::now();
//       } else {
//         msg.header.stamp = ros::Time(time_in_ms / 1000.0);
//       }
//       msg.header.frame_id = m_tf_prefix + "/base_link";
//       msg.orientation_covariance[0] = -1;

//       // measured in deg/s; need to convert to rad/s
//       msg.angular_velocity.x = degToRad(data->gyro_x);
//       msg.angular_velocity.y = degToRad(data->gyro_y);
//       msg.angular_velocity.z = degToRad(data->gyro_z);

//       // measured in mG; need to convert to m/s^2
//       msg.linear_acceleration.x = data->acc_x * 9.81;
//       msg.linear_acceleration.y = data->acc_y * 9.81;
//       msg.linear_acceleration.z = data->acc_z * 9.81;

//       m_pubImu.publish(msg);
//     }
//   }

//   void onLog2Data(uint32_t time_in_ms, log2* data) {

//     if (m_enable_logging_temperature) {
//       sensor_msgs::Temperature msg;
//       if (m_use_ros_time) {
//         msg.header.stamp = ros::Time::now();
//       } else {
//         msg.header.stamp = ros::Time(time_in_ms / 1000.0);
//       }
//       msg.header.frame_id = m_tf_prefix + "/base_link";
//       // measured in degC
//       msg.temperature = data->baro_temp;
//       m_pubTemp.publish(msg);
//     }

//     if (m_enable_logging_magnetic_field) {
//       sensor_msgs::MagneticField msg;
//       if (m_use_ros_time) {
//         msg.header.stamp = ros::Time::now();
//       } else {
//         msg.header.stamp = ros::Time(time_in_ms / 1000.0);
//       }
//       msg.header.frame_id = m_tf_prefix + "/base_link";

//       // measured in Tesla
//       msg.magnetic_field.x = data->mag_x;
//       msg.magnetic_field.y = data->mag_y;
//       msg.magnetic_field.z = data->mag_z;
//       m_pubMag.publish(msg);
//     }

//     if (m_enable_logging_pressure) {
//       std_msgs::Float32 msg;
//       // hPa (=mbar)
//       msg.data = data->baro_pressure;
//       m_pubPressure.publish(msg);
//     }

//     if (m_enable_logging_battery) {
//       std_msgs::Float32 msg;
//       // V
//       msg.data = data->pm_vbat;
//       m_pubBattery.publish(msg);
//     }
//   }

//   void onPoseData(uint32_t time_in_ms, logPose* data) {
//     if (m_enable_logging_pose) {
//       geometry_msgs::PoseStamped msg;
//       if (m_use_ros_time) {
//         msg.header.stamp = ros::Time::now();
//       } else {
//         msg.header.stamp = ros::Time(time_in_ms / 1000.0);
//       }
//       msg.header.frame_id = m_tf_prefix + "/base_link";

//       msg.pose.position.x = data->x;
//       msg.pose.position.y = data->y;
//       msg.pose.position.z = data->z;

//       float q[4];
//       quatdecompress(data->quatCompressed, q);
//       msg.pose.orientation.x = q[0];
//       msg.pose.orientation.y = q[1];
//       msg.pose.orientation.z = q[2];
//       msg.pose.orientation.w = q[3];

//       m_pubPose.publish(msg);
//     }
//   }

//   void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

//     ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

//     crazyflie_driver::GenericLogData msg;
//     if (m_use_ros_time) {
//       msg.header.stamp = ros::Time::now();
//     } else {
//       msg.header.stamp = ros::Time(time_in_ms / 1000.0);
//     }
//     msg.header.frame_id = m_tf_prefix + "/base_link";
//     msg.values = *values;

//     pub->publish(msg);
//   }

//   void onEmptyAck(const crtpPlatformRSSIAck* data) {
//       std_msgs::Float32 msg;
//       // dB
//       msg.data = data->rssi;
//       m_pubRssi.publish(msg);
//   }

//   void onLinkQuality(float linkQuality) {
//       if (linkQuality < 0.7) {
//         ROS_WARN_NAMED(m_tf_prefix, "Link Quality low (%f)", linkQuality);
//       }
//   }

//   void onConsole(const char* msg) {
//     static std::string messageBuffer;
//     messageBuffer += msg;
//     size_t pos = messageBuffer.find('\n');
//     if (pos != std::string::npos) {
//       messageBuffer[pos] = 0;
//       ROS_INFO_NAMED(m_tf_prefix, "CF Console: %s", messageBuffer.c_str());
//       messageBuffer.erase(0, pos+1);
//     }
//   }

//   void onGenericPacket(const ITransport::Ack& ack) {
//     crazyflie_driver::crtpPacket packet;
//     packet.size = ack.size;
//     packet.header = ack.data[0];
//     memcpy(&packet.data[0], &ack.data[1], ack.size);
//     m_pubPackets.publish(packet);
//   }

//   bool setGroupMask(
//     crazyflie_driver::SetGroupMask::Request& req,
//     crazyflie_driver::SetGroupMask::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "SetGroupMask requested");
//     m_cf.setGroupMask(req.groupMask);
//     return true;
//   }

//   bool takeoff(
//     crazyflie_driver::Takeoff::Request& req,
//     crazyflie_driver::Takeoff::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "Takeoff requested");
//     m_cf.takeoff(req.height, req.duration.toSec(), req.groupMask);
//     return true;
//   }

//   bool land(
//     crazyflie_driver::Land::Request& req,
//     crazyflie_driver::Land::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "Land requested");
//     m_cf.land(req.height, req.duration.toSec(), req.groupMask);
//     return true;
//   }

//   bool stop(
//     crazyflie_driver::Stop::Request& req,
//     crazyflie_driver::Stop::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "Stop requested");
//     m_cf.stop(req.groupMask);
//     return true;
//   }

//   bool goTo(
//     crazyflie_driver::GoTo::Request& req,
//     crazyflie_driver::GoTo::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "GoTo requested");
//     m_cf.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.relative, req.groupMask);
//     return true;
//   }

//   bool uploadTrajectory(
//     crazyflie_driver::UploadTrajectory::Request& req,
//     crazyflie_driver::UploadTrajectory::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "UploadTrajectory requested");

//     std::vector<Crazyflie::poly4d> pieces(req.pieces.size());
//     for (size_t i = 0; i < pieces.size(); ++i) {
//       if (   req.pieces[i].poly_x.size() != 8
//           || req.pieces[i].poly_y.size() != 8
//           || req.pieces[i].poly_z.size() != 8
//           || req.pieces[i].poly_yaw.size() != 8) {
//         ROS_FATAL_NAMED(m_tf_prefix, "Wrong number of pieces!");
//         return false;
//       }
//       pieces[i].duration = req.pieces[i].duration.toSec();
//       for (size_t j = 0; j < 8; ++j) {
//         pieces[i].p[0][j] = req.pieces[i].poly_x[j];
//         pieces[i].p[1][j] = req.pieces[i].poly_y[j];
//         pieces[i].p[2][j] = req.pieces[i].poly_z[j];
//         pieces[i].p[3][j] = req.pieces[i].poly_yaw[j];
//       }
//     }
//     m_cf.uploadTrajectory(req.trajectoryId, req.pieceOffset, pieces);

//     ROS_INFO_NAMED(m_tf_prefix, "Upload completed!");
//     return true;
//   }

//   bool startTrajectory(
//     crazyflie_driver::StartTrajectory::Request& req,
//     crazyflie_driver::StartTrajectory::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "StartTrajectory requested");
//     m_cf.startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.relative, req.groupMask);
//     return true;
//   }

//   bool notifySetpointsStop(
//     crazyflie_driver::NotifySetpointsStop::Request& req,
//     crazyflie_driver::NotifySetpointsStop::Response& res)
//   {
//     ROS_INFO_NAMED(m_tf_prefix, "NotifySetpointsStop requested");
//     m_cf.notifySetpointsStop(req.remainValidMillisecs);
//     return true;
//   }

// private:
//   std::string m_tf_prefix;
//   Crazyflie m_cf;
//   bool m_isEmergency;
//   float m_roll_trim;
//   float m_pitch_trim;
//   bool m_enableLogging;
//   bool m_enableParameters;
//   std::vector<crazyflie_driver::LogBlock> m_logBlocks;
//   bool m_use_ros_time;
//   bool m_enable_logging_imu;
//   bool m_enable_logging_temperature;
//   bool m_enable_logging_magnetic_field;
//   bool m_enable_logging_pressure;
//   bool m_enable_logging_battery;
//   bool m_enable_logging_pose;
//   bool m_enable_logging_packets;

//   ros::ServiceServer m_serviceEmergency;
//   ros::ServiceServer m_serviceUpdateParams;
//   ros::ServiceServer m_sendPacketServer;

//   // High-level setpoints
//   ros::ServiceServer m_serviceSetGroupMask;
//   ros::ServiceServer m_serviceTakeoff;
//   ros::ServiceServer m_serviceLand;
//   ros::ServiceServer m_serviceStop;
//   ros::ServiceServer m_serviceGoTo;
//   ros::ServiceServer m_serviceUploadTrajectory;
//   ros::ServiceServer m_serviceStartTrajectory;
//   ros::ServiceServer m_serviceNotifySetpointsStop;

//   ros::Subscriber m_subscribeCmdVel;
//   ros::Subscriber m_subscribeCmdFullState;
//   ros::Subscriber m_subscribeCmdHover;
//   ros::Subscriber m_subscribeCmdStop;
//   ros::Subscriber m_subscribeCmdPosition;
//   ros::Subscriber m_subscribeExternalPosition;
//   ros::Subscriber m_subscribeExternalPose;
//   ros::Subscriber m_subscribeCmdVelocityWorld;
//   ros::Publisher m_pubImu;
//   ros::Publisher m_pubTemp;
//   ros::Publisher m_pubMag;
//   ros::Publisher m_pubPressure;
//   ros::Publisher m_pubBattery;
//   ros::Publisher m_pubPose;
//   ros::Publisher m_pubPackets;
//   ros::Publisher m_pubRssi;
//   std::vector<ros::Publisher> m_pubLogDataGeneric;

//   bool m_sentSetpoint, m_sentExternalPosition;

//   std::thread m_thread;
//   ros::CallbackQueue m_callback_queue;
// };

// class CrazyflieServer
// {
// public:
//   CrazyflieServer()
//   {

//   }

//   void run()
//   {
//     ros::NodeHandle n;
//     ros::CallbackQueue callback_queue;
//     n.setCallbackQueue(&callback_queue);

//     ros::ServiceServer serviceAdd = n.advertiseService("add_crazyflie", &CrazyflieServer::add_crazyflie, this);
//     ros::ServiceServer serviceRemove = n.advertiseService("remove_crazyflie", &CrazyflieServer::remove_crazyflie, this);

//     // // High-level API
//     // ros::ServiceServer serviceTakeoff = n.advertiseService("takeoff", &CrazyflieServer::takeoff, this);
//     // ros::ServiceServer serviceLand = n.advertiseService("land", &CrazyflieROS::land, this);
//     // ros::ServiceServer serviceStop = n.advertiseService("stop", &CrazyflieROS::stop, this);
//     // ros::ServiceServer serviceGoTo = n.advertiseService("go_to", &CrazyflieROS::goTo, this);
//     // ros::ServiceServer startTrajectory = n.advertiseService("start_trajectory", &CrazyflieROS::startTrajectory, this);

//     while(ros::ok()) {
//       // Execute any ROS related functions now
//       callback_queue.callAvailable(ros::WallDuration(0.0));
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
//   }

// private:

//   bool add_crazyflie(
//     crazyflie_driver::AddCrazyflie::Request  &req,
//     crazyflie_driver::AddCrazyflie::Response &res)
//   {
//     ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
//       req.uri.c_str(),
//       req.tf_prefix.c_str(),
//       req.roll_trim,
//       req.pitch_trim,
//       req.enable_parameters,
//       req.enable_logging,
//       req.use_ros_time);

//     // Ignore if the uri is already in use
//     if (m_crazyflies.find(req.uri) != m_crazyflies.end()) {
//       ROS_ERROR("Cannot add %s, already added.", req.uri.c_str());
//       return false;
//     }

//     CrazyflieROS* cf = new CrazyflieROS(
//       req.uri,
//       req.tf_prefix,
//       req.roll_trim,
//       req.pitch_trim,
//       req.enable_logging,
//       req.enable_parameters,
//       req.log_blocks,
//       req.use_ros_time,
//       req.enable_logging_imu,
//       req.enable_logging_temperature,
//       req.enable_logging_magnetic_field,
//       req.enable_logging_pressure,
//       req.enable_logging_battery,
//       req.enable_logging_pose,
//       req.enable_logging_packets);

//     m_crazyflies[req.uri] = cf;

//     return true;
//   }

//   bool remove_crazyflie(
//     crazyflie_driver::RemoveCrazyflie::Request  &req,
//     crazyflie_driver::RemoveCrazyflie::Response &res)
//   {

//     if (m_crazyflies.find(req.uri) == m_crazyflies.end()) {
//       ROS_ERROR("Cannot remove %s, not connected.", req.uri.c_str());
//       return false;
//     }

//     ROS_INFO("Removing crazyflie at uri %s.", req.uri.c_str());

//     m_crazyflies[req.uri]->stop();
//     delete m_crazyflies[req.uri];
//     m_crazyflies.erase(req.uri);

//     ROS_INFO("Crazyflie %s removed.", req.uri.c_str());

//     return true;
//   }

//   // bool takeoff(
//   //   crazyflie_driver::Takeoff::Request& req,
//   //   crazyflie_driver::Takeoff::Response& res)
//   // {
//   //   ROS_INFO("Takeoff requested");
//   //   m_cfbc.takeoff(req.height, req.duration.toSec(), req.groupMask);
//   //   return true;
//   // }

//   // bool land(
//   //   crazyflie_driver::Land::Request& req,
//   //   crazyflie_driver::Land::Response& res)
//   // {
//   //   ROS_INFO("Land requested");
//   //   m_cfbc.land(req.height, req.duration.toSec(), req.groupMask);
//   //   return true;
//   // }

//   // bool stop(
//   //   crazyflie_driver::Stop::Request& req,
//   //   crazyflie_driver::Stop::Response& res)
//   // {
//   //   ROS_INFO("Stop requested");
//   //   m_cfbc.stop(req.groupMask);
//   //   return true;
//   // }

//   // bool goTo(
//   //   crazyflie_driver::GoTo::Request& req,
//   //   crazyflie_driver::GoTo::Response& res)
//   // {
//   //   ROS_INFO("GoTo requested");
//   //   // this is always relative
//   //   m_cfbc.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.groupMask);
//   //   return true;
//   // }

//   // bool startTrajectory(
//   //   crazyflie_driver::StartTrajectory::Request& req,
//   //   crazyflie_driver::StartTrajectory::Response& res)
//   // {
//   //   ROS_INFO("StartTrajectory requested");
//   //   // this is always relative
//   //   m_cfbc.startTrajectory(req.index, req.numPieces, req.timescale, req.reversed, req.groupMask);
//   //   return true;
//   // }

// private:
//   std::map<std::string, CrazyflieROS*> m_crazyflies;
// };




int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");

  CrazyflieServer cfserver;
  cfserver.run();

  return 0;
}


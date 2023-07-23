#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/NotifySetpointsStop.h"
#include "crazyflie_driver/RemoveCrazyflie.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/sendPacket.h"

#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/Full.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/crtpPacket.h"
#include "crazyflie_cpp/Crazyradio.h"
#include "crazyflie_cpp/crtp.h"
#include "std_srvs/Empty.h"
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"

//#include <regex>
#include <thread>
#include <mutex>

#include <string>
#include <map>

#include <crazyflie_cpp/Crazyflie.h>

constexpr double pi() { return 3.141592653589793238462643383279502884; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

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
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_pose,
    bool enable_logging_packets)
    : m_tf_prefix(tf_prefix)
    , m_cf(
      link_uri,
      rosLogger,
      std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1))
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_pose(enable_logging_pose)
    , m_enable_logging_packets(enable_logging_packets)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_serviceSetGroupMask()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceStop()
    , m_serviceGoTo()
    , m_serviceNotifySetpointsStop()
    , m_subscribeCmdVel()
    , m_subscribeCmdFull()
    , m_subscribeCmdStop()
    , m_subscribeCmdPosition()
    , m_subscribeExternalPosition()
    , m_pubRssi()
    , m_sentSetpoint(false)
    , m_sentExternalPosition(false)
  {
    m_thread = std::thread(&CrazyflieROS::run, this);
  }

  void stop()
  {
    ROS_INFO_NAMED(m_tf_prefix, "Disconnecting ...");
    m_isEmergency = true;
    m_thread.join();
  }

  /**
   * Service callback which transmits a packet to the crazyflie
   * @param  req The service request, which contains a crtpPacket to transmit.
   * @param  res The service response, which is not used.
   * @return     returns true always
   */
  bool sendPacket (
    crazyflie_driver::sendPacket::Request &req,
    crazyflie_driver::sendPacket::Response &res)
  {
    /** Convert the message struct to the packet struct */
    crtpPacket_t packet;
    packet.size = req.packet.size;
    packet.header = req.packet.header;
    for (int i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
      packet.data[i] = req.packet.data[i];
    }
    m_cf.queueOutgoingPacket(packet);
    return true;
  }

private:
  struct logPose {
    int16_t x;
    int16_t y;
    int16_t z;
    int32_t quatCompressed;
  } __attribute__((packed));

  struct logSignals {
    float x;
    float y;
    float z;
    float u;
  } __attribute__((packed));

private:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL_NAMED(m_tf_prefix, "Emergency requested!");
    m_isEmergency = true;
    m_cf.emergencyStop();

    return true;
  }

  template<class T, class U>
  void updateParam(uint16_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

void cmdStop(
    const std_msgs::Empty::ConstPtr& msg)
  {
     //ROS_INFO("got a stop setpoint");
    if (!m_isEmergency) {
      m_cf.sendStop();
      m_sentSetpoint = true;
      //ROS_INFO("set a stop setpoint");
    }
  }

void cmdPositionSetpoint(
    const crazyflie_driver::Position::ConstPtr& msg)
  {
    if(!m_isEmergency) {
      float x = msg->x;
      float y = msg->y;
      float z = msg->z;
      float yaw = msg->yaw;

      m_cf.sendPositionSetpoint(x, y, z, yaw);
      m_sentSetpoint = true;
    }
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "Update parameters");
    for (auto&& p : req.params) 
    {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) 
        {
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
      else 
      {
        ROS_ERROR_NAMED(m_tf_prefix, "Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

    void updateParams(
    CrazyflieROS* cf)
  {
    ros::NodeHandle n("~");
    ros::NodeHandle nGlobal;

    // update CF-specific parameters

    std::vector<XmlRpc::XmlRpcValue> firmwareParamsVec(2);
    n.getParam("firmwareParams", firmwareParamsVec[0]);
    nGlobal.getParam("ControllerParams/firmwareParams", firmwareParamsVec[1]);

    crazyflie_driver::UpdateParams::Request request;
    crazyflie_driver::UpdateParams::Response response;

    for (auto& firmwareParams : firmwareParamsVec) 
    {
      auto iter = firmwareParams.begin();
      for (; iter != firmwareParams.end(); ++iter) 
      {
        std::string group = iter->first;
        XmlRpc::XmlRpcValue v = iter->second;
        auto iter2 = v.begin();
        for (; iter2 != v.end(); ++iter2) 
        {
          std::string param = iter2->first;
          XmlRpc::XmlRpcValue value = iter2->second;
          if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) 
          {
            bool b = value;
            nGlobal.setParam(m_tf_prefix + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } 
          else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) 
          {
            int b = value;
            nGlobal.setParam(m_tf_prefix + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } 
          else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) 
          {
            double b = value;
            nGlobal.setParam(m_tf_prefix + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } 
          else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) 
          {
            // "1e-5" is not recognize as double; convert manually here
            std::string value_str = value;
            double value = std::stod(value_str);
            nGlobal.setParam(m_tf_prefix + "/" + group + "/" + param, value);
            std::cout << "update " << group + "/" + param << " to " << value << std::endl;
          } 
          else 
          {
            ROS_ERROR("No known type for %s.%s! (type: %d)", group.c_str(), param.c_str(), value.getType());
          }
          request.params.push_back(group + "/" + param);

        }
      }
    }
    cf->updateParams(request, response);
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      m_sentSetpoint = true;
    }
  }


  void cmdFullSetpoint(
    const crazyflie_driver::Full::ConstPtr& msg)
  {
    //ROS_INFO("got a full state setpoint");
    if (!m_isEmergency) {
      float x = msg->twist1.linear.x;
      float y = msg->twist1.linear.y;
      float z = msg->twist1.linear.z;
      float vx = msg->twist1.angular.x;
      float vy = msg->twist1.angular.y;
      float vz = msg->twist1.angular.z;
      float ax = msg->twist2.linear.x;
      float ay = msg->twist2.linear.y;
      float az = msg->twist2.linear.z;
      float psi = msg->twist2.angular.x;
      // m_cf.sendFullSetpoint(
      //   x, y, z,
      //   vx, vy, vz,
      //   ax, ay, az,
      //   psi);

      m_cf.sendPositionSetpoint(
        x, y, z, psi);
      m_sentSetpoint = true;
      //ROS_INFO("set a full setpoint");
    }
  }


  void positionMeasurementChanged(
    const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    m_cf.sendExternalPositionUpdate(msg->point.x, msg->point.y, msg->point.z);
    m_sentExternalPosition = true;
  }

  void poseMeasurementChanged(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    m_cf.sendExternalPoseUpdate(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    m_sentExternalPosition = true;
  }

  void run()
  {
    ros::NodeHandle n;
    n.setCallbackQueue(&m_callback_queue);

    m_subscribeCmdVel = n.subscribe(m_tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_subscribeCmdFull = n.subscribe(m_tf_prefix + "/cmd_full", 1, &CrazyflieROS::cmdFullSetpoint, this);
    m_subscribeExternalPosition = n.subscribe(m_tf_prefix + "/external_position", 1, &CrazyflieROS::positionMeasurementChanged, this);
    m_subscribeExternalPose = n.subscribe(m_tf_prefix + "/external_pose", 1, &CrazyflieROS::poseMeasurementChanged, this);
    m_serviceEmergency = n.advertiseService(m_tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
    m_subscribeCmdPosition = n.subscribe(m_tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);


    m_serviceSetGroupMask = n.advertiseService(m_tf_prefix + "/set_group_mask", &CrazyflieROS::setGroupMask, this);
    m_serviceTakeoff = n.advertiseService(m_tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
    m_serviceLand = n.advertiseService(m_tf_prefix + "/land", &CrazyflieROS::land, this);
    m_serviceStop = n.advertiseService(m_tf_prefix + "/stop", &CrazyflieROS::stop, this);
    m_serviceGoTo = n.advertiseService(m_tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
    m_serviceNotifySetpointsStop = n.advertiseService(m_tf_prefix + "/notify_setpoints_stop", &CrazyflieROS::notifySetpointsStop, this);



    if (m_enable_logging_pose) {
      m_pubPose = n.advertise<geometry_msgs::PoseStamped>(m_tf_prefix + "/pose", 10);
    }
    m_pubSignals = n.advertise<geometry_msgs::Twist>(m_tf_prefix + "/sc", 10);
    if (m_enable_logging_packets) {
      m_pubPackets = n.advertise<crazyflie_driver::crtpPacket>(m_tf_prefix + "/packets", 10);
      std::function<void(const ITransport::Ack&)> cb_genericPacket = std::bind(&CrazyflieROS::onGenericPacket, this, std::placeholders::_1);
      m_cf.setGenericPacketCallback(cb_genericPacket);
    }
    m_pubRssi = n.advertise<std_msgs::Float32>(m_tf_prefix + "/rssi", 10);

    m_sendPacketServer = n.advertiseService(m_tf_prefix + "/send_packet"  , &CrazyflieROS::sendPacket, this);

    // m_cf.reboot();

    auto start = std::chrono::system_clock::now();

    m_cf.logReset();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    if (m_enableParameters)
    {
      ROS_INFO_NAMED(m_tf_prefix, "Requesting parameters...");
      m_cf.requestParamToc();
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
      }
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }

    std::unique_ptr<LogBlock<logPose> > logBlockPose;
    std::unique_ptr<LogBlock<logSignals> > logBlockSignals;
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO_NAMED(m_tf_prefix, "Requesting Logging variables...");
      m_cf.requestLogToc();


        std::function<void(uint32_t, logSignals*)> cb = std::bind(&CrazyflieROS::onPoseData1, this, std::placeholders::_1, std::placeholders::_2);

        logBlockSignals.reset(new LogBlock<logSignals>(
          &m_cf,{
            {"signals_n", "tau_phi"},
            {"signals_n", "tau_theta"},
            {"signals_n", "tau_psi"},
            {"signals_n", "u"}
          }, cb));
        logBlockSignals->start(1); // 10ms


      if (m_enable_logging_pose) {
        std::function<void(uint32_t, logPose*)> cb = std::bind(&CrazyflieROS::onPoseData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockPose.reset(new LogBlock<logPose>(
          &m_cf,{
            {"stateEstimate", "x"},
            {"stateEstimate", "y"},
            {"stateEstimate", "z"},
            {"stateEstimateZ", "quat"}
          }, cb));
        logBlockPose->start(1); // 10ms
      }
    }

    ROS_INFO_NAMED(m_tf_prefix, "Requesting memories...");
    m_cf.requestMemoryToc();

    ROS_INFO_NAMED(m_tf_prefix, "Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO_NAMED(m_tf_prefix, "Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint && !m_sentExternalPosition) {
        m_cf.transmitPackets();
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      m_sentExternalPosition = false;

      // Execute any ROS related functions now
      m_callback_queue.callAvailable(ros::WallDuration(0.0));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

  void onPoseData(uint32_t time_in_ms, logPose* data) {
    if (m_enable_logging_pose) {
      geometry_msgs::PoseStamped msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

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
    }
  }

    void onPoseData1(uint32_t time_in_ms, logSignals* data) {
      geometry_msgs::Twist msg;
      msg.linear.x = data->x;
      msg.linear.y = data->y;
      msg.linear.z = data->z;
      msg.angular.x = data->u;
      m_pubSignals.publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN_NAMED(m_tf_prefix, "Link Quality low (%f)", linkQuality);
      }
  }

  void onConsole(const char* msg) {
    static std::string messageBuffer;
    messageBuffer += msg;
    size_t pos = messageBuffer.find('\n');
    if (pos != std::string::npos) {
      messageBuffer[pos] = 0;
      ROS_INFO_NAMED(m_tf_prefix, "CF Console: %s", messageBuffer.c_str());
      messageBuffer.erase(0, pos+1);
    }
  }
  void onGenericPacket(const ITransport::Ack& ack) {
    crazyflie_driver::crtpPacket packet;
    packet.size = ack.size;
    packet.header = ack.data[0];
    memcpy(&packet.data[0], &ack.data[1], ack.size);
    m_pubPackets.publish(packet);
  }

  bool setGroupMask(
    crazyflie_driver::SetGroupMask::Request& req,
    crazyflie_driver::SetGroupMask::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "SetGroupMask requested");
    m_cf.setGroupMask(req.groupMask);
    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "Takeoff requested");
    m_cf.takeoff(req.height, req.duration.toSec(), req.groupMask);
    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "Land requested");
    m_cf.land(req.height, req.duration.toSec(), req.groupMask);
    return true;
  }

  bool stop(
    crazyflie_driver::Stop::Request& req,
    crazyflie_driver::Stop::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "Stop requested");
    m_cf.stop(req.groupMask);
    return true;
  }

  bool goTo(
    crazyflie_driver::GoTo::Request& req,
    crazyflie_driver::GoTo::Response& res)
  {
    ROS_INFO_NAMED(m_tf_prefix, "GoTo requested");
    m_cf.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.relative, req.groupMask);
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

private:
  std::string m_tf_prefix;
  Crazyflie m_cf;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;
  bool m_enableParameters;
  bool m_use_ros_time;
  bool m_enable_logging_pose;
  bool m_enable_logging_packets;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_sendPacketServer;

  // High-level setpoints
  ros::ServiceServer m_serviceSetGroupMask;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceStop;
  ros::ServiceServer m_serviceGoTo;
  ros::ServiceServer m_serviceNotifySetpointsStop;

  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeCmdFull;
  ros::Subscriber m_subscribeCmdStop;
  ros::Subscriber m_subscribeCmdPosition;
  ros::Subscriber m_subscribeExternalPosition;
  ros::Subscriber m_subscribeExternalPose;
  ros::Publisher m_pubPose;
  ros::Publisher m_pubSignals;
  ros::Publisher m_pubPackets;
  ros::Publisher m_pubRssi;
  std::vector<ros::Publisher> m_pubLogDataGeneric;

  bool m_sentSetpoint, m_sentExternalPosition;

  std::thread m_thread;
  ros::CallbackQueue m_callback_queue;
};

class CrazyflieServer
{
public:
  CrazyflieServer()
  {

  }

  void run()
  {
    ros::NodeHandle n;
    ros::CallbackQueue callback_queue;
    n.setCallbackQueue(&callback_queue);

    ros::ServiceServer serviceAdd = n.advertiseService("add_crazyflie", &CrazyflieServer::add_crazyflie, this);
    ros::ServiceServer serviceRemove = n.advertiseService("remove_crazyflie", &CrazyflieServer::remove_crazyflie, this);

    while(ros::ok()) {
      // Execute any ROS related functions now
      callback_queue.callAvailable(ros::WallDuration(0.0));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

private:
  std::string m_tf_prefix;
  bool add_crazyflie(
    crazyflie_driver::AddCrazyflie::Request  &req,
    crazyflie_driver::AddCrazyflie::Response &res)
  {
    ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
      req.uri.c_str(),
      req.tf_prefix.c_str(),
      req.roll_trim,
      req.pitch_trim,
      req.enable_parameters,
      req.enable_logging,
      req.use_ros_time);

    // Ignore if the uri is already in use
    if (m_crazyflies.find(req.uri) != m_crazyflies.end()) 
    {
      ROS_ERROR("Cannot add %s, already added.", req.uri.c_str());
      return false;
    }

    CrazyflieROS* cf = new CrazyflieROS(
      req.uri,
      req.tf_prefix,
      req.roll_trim,
      req.pitch_trim,
      req.enable_logging,
      req.enable_parameters,
      req.log_blocks,
      req.use_ros_time,
      req.enable_logging_pose,
      req.enable_logging_packets);

    m_crazyflies[req.uri] = cf;

    return true;
  }

  bool remove_crazyflie(
    crazyflie_driver::RemoveCrazyflie::Request  &req,
    crazyflie_driver::RemoveCrazyflie::Response &res)
  {

    if (m_crazyflies.find(req.uri) == m_crazyflies.end()) {
      ROS_ERROR("Cannot remove %s, not connected.", req.uri.c_str());
      return false;
    }

    ROS_INFO("Removing crazyflie at uri %s.", req.uri.c_str());

    m_crazyflies[req.uri]->stop();
    delete m_crazyflies[req.uri];
    m_crazyflies.erase(req.uri);

    ROS_INFO("Crazyflie %s removed.", req.uri.c_str());

    return true;
  }

private:
  std::map<std::string, CrazyflieROS*> m_crazyflies;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");

  CrazyflieServer cfserver;

  ros::NodeHandle nGlobal;

  cfserver.run();

  return 0;
}



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class LowPassFilter
{
public:
  LowPassFilter(float alpha) : alpha_(alpha), initialized_(false), prev_output_(0) {}

  float filter(float input)
  {
    if (!initialized_)
    {
      prev_output_ = input;
      initialized_ = true;
    }

    float output = alpha_ * input + (1 - alpha_) * prev_output_;
    prev_output_ = output;

    return output;
  }

private:
  float alpha_; // coeficiente de filtro
  bool initialized_; // indicador de si el filtro ha sido inicializado
  float prev_output_; // valor de salida previo del filtro
};

class VelFilterNode
{
public:
  VelFilterNode() : nh_("~")
  {
    // Obtener el coeficiente de filtro de los parámetros del nodo
    nh_.param<float>("filter_coeff", filter_coeff_, 0.1);

    // Suscribirse al topic de \cmd_vel
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &VelFilterNode::cmdVelCallback, this);

    // Publicar en el topic de \cmd_vel filtrado
    filtered_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("filtered_cmd_vel", 1);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // Filtrar la velocidad lineal y angular de la señal de entrada
    float filtered_linear_vel = linear_vel_filter_.filter(msg->linear.x);
    float filtered_angular_vel = angular_vel_filter_.filter(msg->angular.z);

    // Crear un nuevo mensaje de Twist con las velocidades filtradas
    geometry_msgs::Twist filtered_cmd_vel;
    filtered_cmd_vel.linear.x = filtered_linear_vel;
    filtered_cmd_vel.angular.z = filtered_angular_vel;

    // Publicar la señal de \cmd_vel filtrada
    filtered_cmd_vel_pub_.publish(filtered_cmd_vel);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher filtered_cmd_vel_pub_;

  float filter_coeff_; // coeficiente de filtro
  LowPassFilter linear_vel_filter_{filter_coeff_}; // filtro de velocidad lineal
  LowPassFilter angular_vel_filter_{filter_coeff_}; // filtro de velocidad angular
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_filter_node");
  VelFilterNode node;
  ros::spin();
  return 0;
}

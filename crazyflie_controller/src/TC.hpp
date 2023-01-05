#pragma once

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class TC
{
public:
    TC(
        float zeta,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_zeta(zeta)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_derivative(0)
        , m_previousTime(ros::Time::now())
    {
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_derivative = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    void setDerivative(float derivative)
    {
        m_derivative = derivative;
    }

    float update(float value, float targetValue)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = value - targetValue;
        float sign_error;
        float sign_errorp;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        if (dt > 0)
        {
        m_derivative+=(m_previousError - error) / dt;
        }
        // Funcion Signo del error
        
        if (error<0)
        {
            sign_error=-1;
        }
        else if (error>=0)
        {
            sign_error=1;
        }

        // Funcion Signo de la derivada del error
        
        if (m_derivative<0)
        {
            sign_errorp=-1;
        }
        else if (m_derivative>=0)
        {
            sign_errorp=1;
        }


        float k1=25*pow(m_zeta,(double)2/3);
        float k2=15*pow(m_zeta,(double)1/2);
        float k3=2.3*m_zeta;
        float k4=1.1*m_zeta; 

        float output = (-k1*pow(fabs(error),(double)1/3)*sign_error-k2*pow(fabs(m_derivative),(double)1/2)*sign_errorp+(-k3*sign_error-k4*sign_errorp)*dt);

        m_previousError = error;
        m_previousTime = time;
        // self.pubOutput.publish(output)
        // self.pubError.publish(error)
        // fprint("%f",output)

        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }
    float control(float value, float targetValue)
    {
     ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = value - targetValue;
        float sign_error;
        float sign_errorp;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        if (dt > 0)
        {
        m_derivative+=(m_previousError - error) / dt;
        }
        // Funcion Signo del error
        
        if (error<0)
        {
            sign_error=-1;
        }
        else if (error>=0)
        {
            sign_error=1;
        }

        // Funcion Signo de la derivada del error
        
        if (m_derivative<0)
        {
            sign_errorp=-1;
        }
        else if (m_derivative>=0)
        {
            sign_errorp=1;
        }


        float k1=25*pow(m_zeta,(double)2/3);
        float k2=15*pow(m_zeta,(double)1/2);
        float k3=2.3*m_zeta;
        float k4=1.1*m_zeta; 

        float output = (-k1*pow(fabs(error),(double)1/3)*sign_error-k2*pow(fabs(m_derivative),(double)1/2)*sign_errorp+(-k3*sign_error-k4*sign_errorp)*dt);

        m_previousError = error;
        m_previousTime = time;


        return(error);

    }
private:
    float m_zeta;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_derivative;
    float m_previousError;
    ros::Time m_previousTime;
};

#include <std_msgs/Float32.h>

class LOWPASS
{
public:
  LOWPASS(float timeC)    
    : tc(timeC)
    , m_previousTime(ros::Time::now())
    , m_lastOutput(0)
    , prevInput(0)
  {
  }

  void reset()
  {
    m_lastOutput = 0;
    m_previousTime = ros::Time::now();
  }

  float filter(float input)
  {
    ros::Time time = ros::Time::now();
    float dt = time.toSec() - m_previousTime.toSec();

    float output = (dt*(input+prevInput) - m_lastOutput*(dt-2*tc)) / (dt + 2*tc);

    prevInput = input;
    m_previousTime = time;
    m_lastOutput = output;
    
    return m_lastOutput;
  }

private:
  float m_lastOutput;
  float prevInput;
  float tc;
  ros::Time m_previousTime;
};

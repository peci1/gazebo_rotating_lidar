#include <gazebo_rotating_lidar/RotatingLidarSensor.h>

#include <boost/algorithm/string.hpp>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
extern "C"
{
  GZ_REGISTER_STATIC_SENSOR("rotating_lidar", RotatingLidarSensor)
}

namespace gazebo
{
namespace sensors
{

class RotatingLidarSensorPrivate
{

};

RotatingLidarSensor::RotatingLidarSensor() : Sensor(sensors::RAY),  //TODO: maybe OTHER?
  dataPtr(new RotatingLidarSensorPrivate)
{
}

void RotatingLidarSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  gzmsg << "Rotating lidar loaded" << std::endl;
}

void RotatingLidarSensor::Init()
{
  Sensor::Init();
}

void RotatingLidarSensor::Fini()
{
  Sensor::Fini();
}

bool RotatingLidarSensor::UpdateImpl(const bool force)
{
  return false;
}

std::string RotatingLidarSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

RotatingLidarSensor::~RotatingLidarSensor() // NOLINT(hicpp-use-equals-default)
{
}

}
}
#ifndef GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H
#define GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H

#include <gazebo/sensors/sensors.hh>

#include <memory>

namespace gazebo
{
namespace sensors
{

class RotatingLidarSensorPrivate;

class GZ_SENSORS_VISIBLE RotatingLidarSensor : public Sensor
{
  public: RotatingLidarSensor();
  public: ~RotatingLidarSensor() override;

  public: void Load(const std::string &_worldName) override;
  public: void Init() override;
  protected: void Fini() override;
  public: std::string Topic() const override;
  protected: bool UpdateImpl(bool force) override;

  private: std::unique_ptr<RotatingLidarSensorPrivate> dataPtr;
};

}
}

#endif //GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H

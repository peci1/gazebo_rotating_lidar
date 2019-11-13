#ifndef GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H
#define GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H

#include <gazebo/sensors/sensors.hh>

#include <memory>

namespace gazebo
{
namespace sensors
{

class RotatingLidarSensorPrivate;

class GZ_SENSORS_VISIBLE RotatingLidarSensor : public RaySensor
{
  public: RotatingLidarSensor();
  public: ~RotatingLidarSensor() override;

  public: void Load(const std::string &_worldName) override;
  public: void Init() override;
  public: virtual void Reset();
  protected: void Fini() override;
  protected: bool UpdateImpl(bool force) override;
  public: std::string Topic() const override;
  public: bool IsActive() const override;

  public: common::Time TimeIncrement() const;
  public: common::Time ScanTime() const;

  private: std::unique_ptr<RotatingLidarSensorPrivate> dataPtr;

  friend class RotatingLidarSensorPrivate;
};

}
}

#endif //GAZEBO_ROTATING_LIDAR_ROTATINGLIDARSENSOR_H

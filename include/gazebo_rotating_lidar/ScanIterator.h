#ifndef GAZEBO_ROTATING_LIDAR_SCANITERATOR_H
#define GAZEBO_ROTATING_LIDAR_SCANITERATOR_H

#include <functional>
#include <memory>
#include <string>
#include <gazebo/common/Time.hh>
#include <ignition/math/Angle.hh>

namespace gazebo
{
namespace sensors
{

struct ScanRay
{
  ignition::math::Angle angle;
  bool valid;
};

class ScanRayFan
{
  private: std::vector<ScanRay> rays;
  private: size_t numValidRays = 0;

  public: ScanRayFan();
  public: ScanRayFan(const ScanRayFan& _other);

  public: ScanRayFan(const ignition::math::Angle& _fanMinAngle,
      const ignition::math::Angle& _fanMaxAngle,
      const ignition::math::Angle& _scanMinAngle,
      const ignition::math::Angle& _scanMaxAngle,
      const ignition::math::Angle& _angleIncrement);

  public: ScanRayFan(const std::vector<ScanRay>::const_iterator& start,
      const std::vector<ScanRay>::const_iterator& end);

  public: const ignition::math::Angle& MinAngle() const;

  public: const ignition::math::Angle& MaxAngle() const;

  public: void AddRay(const ScanRay& ray);

  public: const decltype(rays)& Rays() const;

  public: bool HasValidRays() const;

  public: bool AllRaysValid() const;

  public: size_t NumRays() const;

  public: size_t NumValidRays() const;
};

class ScanIterator
{
  typedef std::function<void(void)> newScanFunc;
  typedef std::function<void(const ScanRayFan&)> updateScanFunc;
  typedef std::function<void(void)> finishScanFunc;
  typedef std::function<void(void)> firstValidScanFunc;

  struct ScanIteratorPrivate;
  private: std::unique_ptr<ScanIteratorPrivate> dataPtr;

  public: newScanFunc newScan = nullptr;
  public: updateScanFunc updateScan = nullptr;
  public: finishScanFunc finishScan = nullptr;
  public: firstValidScanFunc firstValidScan = nullptr;

  public: ScanIterator(const ignition::math::Angle& _minAngle,
      const ignition::math::Angle& _maxAngle,
      const ignition::math::Angle& _angleIncrement,
      const gazebo::common::Time& _timeIncrement,
      const gazebo::common::Time& _updateDuration);
  virtual ~ScanIterator();

  public: virtual size_t NumValidRaysInWholeScan() const;
  public: virtual size_t NumRaysInWholeScan() const;
  public: virtual size_t MaxNumRaysInSubscan() const;
  public: virtual void Update();
  public: virtual void Reset();

  protected: virtual void OnNewScan();
  protected: virtual void OnValidScan();
  protected: virtual void OnUpdate(const ScanRayFan& subscan);
  protected: virtual void OnScanFinished();
  protected: virtual bool CheckSkip();
};
}
}

#endif //GAZEBO_ROTATING_LIDAR_SCANITERATOR_H

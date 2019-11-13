#include <gazebo_rotating_lidar/ScanIterator.h>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Console.hh>

using ignition::math::Angle;
using gazebo::common::Time;

namespace gazebo
{
namespace sensors
{

ScanRayFan::ScanRayFan() = default;

ScanRayFan::ScanRayFan(const ScanRayFan& _other)
{
  this->rays = _other.rays;
  this->numValidRays = _other.numValidRays;
}

ScanRayFan::ScanRayFan(const Angle& _fanMinAngle, const Angle& _fanMaxAngle,
                       const Angle& _scanMinAngle, const Angle& _scanMaxAngle,
                       const Angle& _angleIncrement)
{
  if (_fanMinAngle > _fanMaxAngle)
  {
    gzthrow("Min angle > max angle");
  }
  if (_scanMinAngle > _scanMaxAngle)
  {
    gzthrow("Min angle > max angle");
  }
  if (_angleIncrement <= 0)
  {
    gzthrow("Angle increment must be positive");
  }

  const Angle angularRange = _fanMaxAngle - _fanMinAngle;
  const double computedNumRays = angularRange.Radian() / _angleIncrement.Radian();
  if (fabs(computedNumRays - round(computedNumRays)) > 1e-2)
  {
    gzthrow("Angular range is not a multiple of angle increment. "
            "computedNumRays = " << computedNumRays);
  }

  const bool wholeCircle = fabs((angularRange - Angle::TwoPi).Radian()) < 1e-6;
  // if this represents the whole circle, we do not include the last ray, but
  // otherwise, we want to include it
  const auto numRays = static_cast<size_t>(
      wholeCircle ? computedNumRays : computedNumRays + 1.0);

  this->rays.reserve(numRays);
  // give the angular bounds some margin to overcome FP roundoff instabilities
  const auto eps = _angleIncrement / 10.0;
  for (size_t i = 0; i < numRays; ++i)
  {
    ScanRay ray {};
    ray.angle = _fanMinAngle + Angle(static_cast<double>(i)) * _angleIncrement;
    ray.valid = ray.angle >= (_scanMinAngle - eps) && ray.angle <= (_scanMaxAngle + eps);
    this->AddRay(ray);
  }
}

ScanRayFan::ScanRayFan(const std::vector<ScanRay>::const_iterator& start,
             const std::vector<ScanRay>::const_iterator& end) : rays(start, end)
{
  for (const auto& ray: this->rays)
  {
    if (ray.valid)
      this->numValidRays += 1;
  }
}

const Angle& ScanRayFan::MinAngle() const
{
  if (this->rays.empty())
    throw std::out_of_range("No rays in ray fan, but asking for min angle");
  return this->rays.front().angle;
}

const Angle& ScanRayFan::MaxAngle() const
{
  if (this->rays.empty())
    throw std::out_of_range("No rays in ray fan, but asking for max angle");
  return this->rays.back().angle;
}

void ScanRayFan::AddRay(const ScanRay& ray)
{
  if (ray.valid)
    this->numValidRays += 1;
  this->rays.push_back(ray);
}

const decltype(ScanRayFan::rays)& ScanRayFan::Rays() const
{
  return this->rays;
}

bool ScanRayFan::HasValidRays() const
{
  return this->numValidRays > 0;
}

bool ScanRayFan::AllRaysValid() const
{
  return this->numValidRays == this->rays.size();
}

size_t ScanRayFan::NumRays() const
{
  return this->rays.size();
}

size_t ScanRayFan::NumValidRays() const
{
  return this->numValidRays;
}

struct ScanIterator::ScanIteratorPrivate
{
  std::unique_ptr<ScanRayFan> scan;
  double firstRay = 0.0;
  double lastRay;
  double raysPerUpdate;
  bool raysPerUpdateAreIntegral;
  size_t updateSkipCount = 0;
  size_t updatesSkipped = 0;
  bool wasValidRay = false;
};

ScanIterator::ScanIterator(const Angle& _minAngle, const Angle& _maxAngle,
                           const Angle& _angleIncrement, const Time& _timeIncrement,
                           const Time& _updateDuration)
                           : dataPtr(new ScanIteratorPrivate)
{
  if (_timeIncrement <= 0)
  {
    gzthrow("Time increment must be positive");
  }
  if (_updateDuration <= 0)
  {
    gzthrow("Update duration must be positive");
  }

  this->dataPtr->scan.reset(new ScanRayFan(_minAngle, _minAngle + Angle::TwoPi,
                                  _minAngle, _maxAngle, _angleIncrement));

  this->dataPtr->raysPerUpdate = (_updateDuration / _timeIncrement).Double();

  if (this->dataPtr->raysPerUpdate > this->dataPtr->scan->NumRays())
  {
    gzthrow("Requesting more than one scan per sensor update is not "
                 "supported. Please, increase sensor update rate.");
  }

  if (this->dataPtr->raysPerUpdate < 1)
  {
    this->dataPtr->updateSkipCount = static_cast<size_t>(round(
        (_timeIncrement / _updateDuration).Double()));
    this->dataPtr->raysPerUpdate = 1.0;
  }
  this->dataPtr->lastRay = this->dataPtr->raysPerUpdate - 1;

  this->dataPtr->raysPerUpdateAreIntegral =
      fabs(this->dataPtr->raysPerUpdate - round(this->dataPtr->raysPerUpdate)) < 1e-3;
  if (!this->dataPtr->raysPerUpdateAreIntegral)
  {
    gzwarn << "Sensor update duration (" << _updateDuration.Double() << ") is "
           << "not a multiple of time increment (" << _timeIncrement.Double()
           << "). Aliasing effects will most probably occur. Consider changing "
           << "the update rate to a multiple of time increment." << std::endl;
  }
}

void ScanIterator::Update()
{
  if (this->CheckSkip())
    return;

  if (round(this->dataPtr->firstRay) < 0 || round(this->dataPtr->lastRay) < 0)
  {
    gzthrow("Scan iterators are invalid: first = "
                << this->dataPtr->firstRay << ", last = " << this->dataPtr->lastRay);
  }

  if (round(this->dataPtr->firstRay) == 0)
    this->OnNewScan();

  const auto firstRayIndex = static_cast<size_t>(round(this->dataPtr->firstRay));
  const auto lastRayIndex = static_cast<size_t>(round(this->dataPtr->lastRay)) + 1;

  if (firstRayIndex >= this->dataPtr->scan->NumRays() || lastRayIndex > this->dataPtr->scan->NumRays())
  {
    gzthrow("Scan iterators are invalid: first = "
                << firstRayIndex << ", last = " << (lastRayIndex - 1)
                << ", num = " << this->dataPtr->scan->NumRays());
  }

  const ScanRayFan subscan(this->dataPtr->scan->Rays().cbegin() + firstRayIndex,
                           this->dataPtr->scan->Rays().cbegin() + lastRayIndex);
  this->OnUpdate(subscan);

  if (round(this->dataPtr->lastRay) >= this->dataPtr->scan->NumRays() - 1)
  {
    this->OnScanFinished();

    const auto lastRay = this->dataPtr->firstRay + this->dataPtr->raysPerUpdate -
        this->dataPtr->scan->NumRays();
    if (round(lastRay) >= 1)
    {
      this->OnNewScan();

      const ScanRayFan excessiveSubscan(
        this->dataPtr->scan->Rays().cbegin(),
        this->dataPtr->scan->Rays().cbegin() + static_cast<size_t>(round(lastRay)));

      this->OnUpdate(excessiveSubscan);
    }
  }

  this->dataPtr->firstRay += this->dataPtr->raysPerUpdate;
  if (round(this->dataPtr->firstRay) > this->dataPtr->scan->NumRays() - 1)
  {
    this->dataPtr->firstRay -= this->dataPtr->scan->NumRays();
  }

  this->dataPtr->lastRay = this->dataPtr->firstRay + this->dataPtr->raysPerUpdate - 1;
  if (round(this->dataPtr->lastRay) >= this->dataPtr->scan->NumRays() - 1)
  {
    this->dataPtr->lastRay = this->dataPtr->scan->NumRays() - 1;
  }

  if (this->dataPtr->raysPerUpdateAreIntegral)
  {
    this->dataPtr->firstRay = round(this->dataPtr->firstRay);
    this->dataPtr->lastRay = round(this->dataPtr->lastRay);
  }
}

void ScanIterator::OnNewScan()
{
  this->dataPtr->wasValidRay = false;
  if (this->newScan != nullptr)
    this->newScan();
}

void ScanIterator::OnValidScan()
{
  if (!this->dataPtr->wasValidRay)
  {
    this->dataPtr->wasValidRay = true;
    if (this->firstValidScan != nullptr)
      this->firstValidScan();
  }
}

void ScanIterator::OnUpdate(const ScanRayFan &subscan)
{
  if (this->updateScan != nullptr)
    this->updateScan(subscan);

  if (subscan.HasValidRays())
    this->OnValidScan();
}

void ScanIterator::OnScanFinished()
{
  if (this->finishScan != nullptr)
    this->finishScan();
}

bool ScanIterator::CheckSkip()
{
  if (this->dataPtr->updateSkipCount <= 1)
    return false;

  ++this->dataPtr->updatesSkipped;

  if (this->dataPtr->updatesSkipped < this->dataPtr->updateSkipCount)
  {
    return true;
  }
  else
  {
    this->dataPtr->updatesSkipped = 0;
    return false;
  }
}

size_t ScanIterator::MaxNumRaysInSubscan() const
{
  return static_cast<size_t>(ceil(this->dataPtr->raysPerUpdate));
}

size_t ScanIterator::NumRaysInWholeScan() const
{
  return this->dataPtr->scan->NumRays();
}

size_t ScanIterator::NumValidRaysInWholeScan() const
{
  return this->dataPtr->scan->NumValidRays();
}

void ScanIterator::Reset()
{
  this->dataPtr->firstRay = 0.0;
  this->dataPtr->lastRay = this->dataPtr->raysPerUpdate - 1;
  this->dataPtr->wasValidRay = false;
  this->dataPtr->updatesSkipped = 0;
}

ScanIterator::~ScanIterator()
{
}

}
}
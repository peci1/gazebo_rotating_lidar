#include <gazebo_rotating_lidar/RotatingLidarSensor.h>
#include <gazebo_rotating_lidar/ScanIterator.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <boost/algorithm/string.hpp>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
using ignition::math::Angle;
using gazebo::common::Time;

extern "C"
{
  GZ_REGISTER_STATIC_SENSOR("ray", RotatingLidarSensor)
}

namespace gazebo
{
namespace sensors
{

typedef std::pair<ignition::math::Vector3d, ignition::math::Vector3d> RayEndpoints;

struct RotatingLidarSensorPrivate
{
  const RotatingLidarSensor* parentObject;

  double minAngle;
  double maxAngle;
  double angleIncrement;
  size_t samples;
  physics::CollisionPtr rayCollision;
  physics::MultiRayShapePtr laserShape;
  std::vector<physics::RayShapePtr> rays;
  std::vector<RayEndpoints> rayOrigEndpoints;
  bool parentLoaded = false;
  transport::PublisherPtr scanPub;
  msgs::LaserScanStamped laserMsg;
  size_t nextHorizontalPosInLaserMsg;
  std::unique_ptr<ScanIterator> scanIterator;
  physics::EntityPtr parentEntity;
  std::mutex mutex;
  common::Time timeIncrement;
  common::Time scanTime;

  event::ConnectionPtr worldResetConnection;
  event::ConnectionPtr timeResetConnection;

  RotatingLidarSensorPrivate(const RotatingLidarSensor* _parentObject)
  {
    this->parentObject = _parentObject;
  }

  void OnNewScan();
  void PublishLaserScan();
  void OnUpdate(const ScanRayFan& subscan);
  void UpdateRays(const ScanRayFan& subscan);
};

RotatingLidarSensor::RotatingLidarSensor() :
  dataPtr(new RotatingLidarSensorPrivate(this))
{
}

void RotatingLidarSensor::Load(const std::string &_worldName)
{
  // Here we need to do some stuff before the parent Load(worldName) function
  // is called. We're in a state that Load(worldName, sdf) has already been
  // called, so this->sdf is already set, but that's about all. We can refer to
  // no other parent members.

  const auto rayElem = this->sdf->GetElement("ray");
  const auto scanElem = rayElem->GetElement("scan");
  const auto horzElem = scanElem->GetElement("horizontal");

  this->dataPtr->minAngle = horzElem->Get<double>("min_angle");
  this->dataPtr->maxAngle = horzElem->Get<double>("max_angle");
  this->dataPtr->samples = horzElem->Get<unsigned int>("samples");
  // this->updatePeriod is only available after Init()
  const auto updateRate = this->sdf->Get<double>("update_rate");
  const auto updateDuration = 1.0 / updateRate;

  // non-SDF config

  // Find a custom_sensor_config_plugin associated to this sensor. It is a
  // container for custom options not included in the SDF spec.
  sdf::ElementPtr configPlugin = nullptr;
  auto plugin = this->sdf->GetElement("plugin");
  while (plugin)
  {
    if (plugin->Get<std::string>("filename") == "libcustom_sensor_config_plugin.so")
    {
      configPlugin = plugin;
      plugin = nullptr;
      break;
    }
    plugin = plugin->GetNextElement("plugin");
  }

  if (configPlugin == nullptr)
  {
    gzerr << "Could not find custom sensor configuration plugin in sensor '"
          << this->sdf->Get<std::string>("name") << "'" << std::endl;
    gzthrow("No custom config plugin found");
  }

  this->dataPtr->timeIncrement = configPlugin->Get<double>("time_increment");
  this->dataPtr->scanTime = configPlugin->Get<double>("scan_time");
  this->dataPtr->angleIncrement = configPlugin->Get<double>("angle_increment");

  this->dataPtr->scanIterator.reset(new ScanIterator(
      Angle(this->dataPtr->minAngle), Angle(this->dataPtr->maxAngle),
      Angle(this->dataPtr->angleIncrement), Time(this->dataPtr->timeIncrement),
      Time(updateDuration)));

  this->dataPtr->scanIterator->newScan = std::bind(
      &RotatingLidarSensorPrivate::OnNewScan, std::ref(this->dataPtr));
  this->dataPtr->scanIterator->firstValidScan = [this] () {
    this->lastMeasurementTime = this->world->SimTime();
    msgs::Set(this->dataPtr->laserMsg.mutable_time(), this->lastMeasurementTime);

    msgs::Set(this->dataPtr->laserMsg.mutable_scan()->mutable_world_pose(),
              this->pose + this->dataPtr->parentEntity->WorldPose());
  };
  this->dataPtr->scanIterator->finishScan = std::bind(
      &RotatingLidarSensorPrivate::PublishLaserScan, std::ref(this->dataPtr));
  this->dataPtr->scanIterator->updateScan = std::bind(
      &RotatingLidarSensorPrivate::OnUpdate, std::ref(this->dataPtr),
      std::placeholders::_1);

  const auto subscanRays = this->dataPtr->scanIterator->MaxNumRaysInSubscan();

  // edit the original SDF so that it represents just the smaller one-update ray
  // fan
  horzElem->GetElement("samples")->Set<size_t>(subscanRays);
  horzElem->GetElement("max_angle")->Set<double>(
      this->dataPtr->minAngle + this->dataPtr->angleIncrement * subscanRays);

  RaySensor::Load(_worldName);
  this->dataPtr->parentLoaded = true;

  {
    // initialize the message fields
    msgs::LaserScan *scan = this->dataPtr->laserMsg.mutable_scan();
    scan->set_angle_min(this->dataPtr->minAngle);
    scan->set_angle_max(this->dataPtr->maxAngle);
    scan->set_angle_step(this->dataPtr->angleIncrement);
    scan->set_count(this->dataPtr->scanIterator->NumValidRaysInWholeScan());

    scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(this->VerticalAngleResolution());
    scan->set_vertical_count(this->VerticalRangeCount());

    scan->set_range_min(this->RangeMin());
    scan->set_range_max(this->RangeMax());
  }

  // Initialize /scan publisher after parent Load() finishes. This allows to use
  // the hack with different topic name for subscans.
  this->dataPtr->scanPub =
      this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

  this->dataPtr->laserShape = this->LaserShape();
  this->dataPtr->rayCollision = boost::dynamic_pointer_cast<physics::Collision>(
      this->dataPtr->laserShape->GetParent());

  this->dataPtr->parentEntity = this->world->EntityByName(this->ParentName());

  const auto rayCount = this->dataPtr->laserShape->RayCount();
  this->dataPtr->rays.reserve(rayCount);
  this->dataPtr->rayOrigEndpoints.reserve(rayCount);
  for (size_t i = 0; i < rayCount; ++i)
  {
    const auto ray = this->dataPtr->laserShape->Ray(i);
    this->dataPtr->rays.push_back(ray);
    std::pair<ignition::math::Vector3d, ignition::math::Vector3d> pair;
    ray->RelativePoints(pair.first, pair.second);
    this->dataPtr->rayOrigEndpoints.push_back(pair);
  }

  gzmsg << "Rotating lidar loaded" << std::endl;
  gzmsg << "time increment: " << this->dataPtr->timeIncrement.Double() << std::endl;
  gzmsg << "angle increment: " << this->dataPtr->angleIncrement << std::endl;
  gzmsg << "angular range: " << this->dataPtr->minAngle << " to " << this->dataPtr->maxAngle << std::endl;
  gzmsg << "Update rate: " << updateRate << std::endl;
  gzmsg << "Update Period: " << updateDuration << std::endl;
  gzmsg << "Scan samples: " << this->dataPtr->scanIterator->NumValidRaysInWholeScan() << "x" << this->VerticalRayCount() << std::endl;
  gzmsg << "Subscan samples: " << subscanRays << "x" << this->VerticalRayCount() << std::endl;
}

std::string RotatingLidarSensor::Topic() const
{
  // This is a little hack to force the parent class to publish to scan_sub, but
  // plugins to subscribe to scan.
  if (!this->dataPtr->parentLoaded)
  {
    return RaySensor::Topic() + "_sub";
  }
  else
  {
    return RaySensor::Topic();
  }
}

void RotatingLidarSensor::Init()
{
  RaySensor::Init();

  this->dataPtr->worldResetConnection = event::Events::ConnectWorldReset(
      std::bind(&RotatingLidarSensor::Reset, this));
  this->dataPtr->timeResetConnection = event::Events::ConnectTimeReset(
      std::bind(&RotatingLidarSensor::Reset, this));

  this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

void RotatingLidarSensor::Fini()
{
  RaySensor::Fini();
}

void RotatingLidarSensorPrivate::UpdateRays(const ScanRayFan& subscan)
{
  const auto angleOffset = subscan.MinAngle().Radian() - this->minAngle;
  const ignition::math::Pose3d poseOffset(0, 0, 0, 0, 0, angleOffset);

  for (size_t i = 0; i < subscan.Rays().size(); ++i)
  {
    if (!subscan.Rays().at(i).valid)
      continue;

    for (size_t j = 0; j < static_cast<size_t>(this->parentObject->VerticalRayCount()); ++j)
    {
      const auto rayIndex = j * this->parentObject->RayCount() + i;
      const auto &ray = this->rays[rayIndex];
      auto start = this->rayOrigEndpoints[rayIndex].first;
      auto end = this->rayOrigEndpoints[rayIndex].second;

      start = poseOffset.CoordPositionAdd(start);
      end = poseOffset.CoordPositionAdd(end);
      ray->SetPoints(start, end);
    }
  }
}

void RotatingLidarSensorPrivate::PublishLaserScan()
{
  if (this->scanPub && this->scanPub->HasConnections())
    this->scanPub->Publish(this->laserMsg);
}

void RotatingLidarSensorPrivate::OnNewScan()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->nextHorizontalPosInLaserMsg = 0;

  msgs::LaserScan *scan = this->laserMsg.mutable_scan();

  scan->clear_ranges();
  scan->clear_intensities();

  const auto numRanges =
      this->scanIterator->NumValidRaysInWholeScan() * this->parentObject->VerticalRangeCount();
  scan->mutable_ranges()->Resize(numRanges, ignition::math::INF_D);
  scan->mutable_intensities()->Resize(numRanges, ignition::math::INF_D);
}

bool RotatingLidarSensor::UpdateImpl(const bool force)
{
  if (this->dataPtr->scanIterator)
    this->dataPtr->scanIterator->Update();

  return true;
}

void RotatingLidarSensorPrivate::OnUpdate(const ScanRayFan& subscan)
{
  // Unfortunately, we had to copy/paste the code from parent class here
  // That's because we can't access laserMsg and we want the most efficient way
  // of getting the generated laserscans

  if (!subscan.HasValidRays())
    return;

  this->UpdateRays(subscan);

  // do the collision checks
  // this eventually call OnNewScans, so move mutex lock behind it in case
  // need to move mutex lock after this? or make the OnNewLaserScan connection
  // call somewhere else?
  this->laserShape->Update();

  // moving this behind laserShape update
  std::lock_guard<std::mutex> lock(this->mutex);

  msgs::LaserScan *scan = this->laserMsg.mutable_scan();

  unsigned int rayCount = this->parentObject->RayCount();
  unsigned int rangeCount = this->parentObject->RangeCount();
  unsigned int verticalRayCount = this->parentObject->VerticalRayCount();
  unsigned int verticalRangeCount = this->parentObject->VerticalRangeCount();

  // Interpolation: for every point in range count, compute interpolated value
  // using four bounding ray samples.
  // (vja, hja)   (vja, hjb)
  //       x---------x
  //       |         |
  //       |    o    |
  //       |         |
  //       x---------x
  // (vjb, hja)   (vjb, hjb)
  // where o: is the range to be interpolated
  //       x: ray sample
  //       vja: is the previous index of ray in vertical direction
  //       vjb: is the next index of ray in vertical direction
  //       hja: is the previous index of ray in horizontal direction
  //       hjb: is the next index of ray in horizontal direction
  unsigned int hja, hjb;
  unsigned int vja = 0, vjb = 0;
  // percentage of interpolation between rays
  double vb = 0, hb;
  // indices of ray samples
  int j1, j2, j3, j4;
  // range values of ray samples
  double r1, r2, r3, r4;

  // Check for the common case of vertical and horizontal resolution being 1,
  // which means that ray count == range count and we can do simple lookup
  // of ranges and intensity data, skipping interpolation.  We could do this
  // check independently for vertical and horizontal, but that's more
  // complexity for an unlikely use case.
  bool interp =
      ((rayCount != rangeCount) || (verticalRayCount != verticalRangeCount));

  size_t addedHorizontalRays = 0;

  // interpolate in vertical direction
  for (unsigned int j = 0; j < verticalRangeCount; ++j)
  {
    addedHorizontalRays = 0;
    if (interp)
    {
      vb = (verticalRangeCount == 1) ? 0 :
           static_cast<double>(j * (verticalRayCount - 1))
               / (verticalRangeCount - 1);
      vja = static_cast<int>(floor(vb));
      vjb = std::min(vja + 1, verticalRayCount - 1);
      vb = vb - floor(vb);

      GZ_ASSERT(vja < verticalRayCount,
                "Invalid vertical ray index used for interpolation");
      GZ_ASSERT(vjb < verticalRayCount,
                "Invalid vertical ray index used for interpolation");
    }
    // interpolate in horizontal direction
    for (unsigned int i = 0; i < rangeCount; ++i)
    {
      double range, intensity;
      if (interp)
      {
        hb = (rangeCount == 1) ? 0 : static_cast<double>(i * (rayCount - 1))
            / (rangeCount - 1);
        hja = static_cast<int>(floor(hb));
        hjb = std::min(hja + 1, rayCount - 1);
        hb = hb - floor(hb);

        GZ_ASSERT(hja < rayCount,
                  "Invalid horizontal ray index used for interpolation");
        GZ_ASSERT(hjb < rayCount,
                  "Invalid horizontal ray index used for interpolation");

        // ignore ranges which contain rays outside the current ray fan
        if (hja >= subscan.Rays().size() || hjb >= subscan.Rays().size())
          continue;

        // ignore ranges with both rays outside the valid range; if only one
        // ray is valid, use only its value
        if (!subscan.Rays().at(hja).valid && !subscan.Rays().at(hjb).valid)
          continue;
        else if (!subscan.Rays().at(hja).valid)
          hb = 1;
        else if (!subscan.Rays().at(hjb).valid)
          hb = 0;

        // indices of 4 corners
        j1 = hja + vja * rayCount;
        j2 = hjb + vja * rayCount;
        j3 = hja + vjb * rayCount;
        j4 = hjb + vjb * rayCount;

        // range readings of 4 corners
        r1 = this->laserShape->GetRange(j1);
        r2 = this->laserShape->GetRange(j2);
        r3 = this->laserShape->GetRange(j3);
        r4 = this->laserShape->GetRange(j4);
        range = (1 - vb) * ((1 - hb) * r1 + hb * r2)
            + vb * ((1 - hb) * r3 + hb * r4);

        // intensity is averaged
        intensity = 0.25 * (this->laserShape->GetRetro(j1)
            + this->laserShape->GetRetro(j2)
            + this->laserShape->GetRetro(j3)
            + this->laserShape->GetRetro(j4));
      }
      else
      {
        if (i >= subscan.NumRays() || !subscan.Rays().at(i).valid)
          continue;

        range = this->laserShape->GetRange(j * rayCount + i);
        intensity = this->laserShape->GetRetro(j * rayCount + i);
      }

      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (range >= this->parentObject->RangeMax())
      {
        range = ignition::math::INF_D;
      }
      else if (range <= this->parentObject->RangeMin())
      {
        range = -ignition::math::INF_D;
      }
      else if (this->parentObject->noises.find(RAY_NOISE) !=
          this->parentObject->noises.end())
      {
        // currently supports only one noise model per laser sensor
        range = this->parentObject->noises.at(RAY_NOISE)->Apply(range);
        range = ignition::math::clamp(range,
                                      this->parentObject->RangeMin(), this->parentObject->RangeMax());
      }

      const auto index = this->nextHorizontalPosInLaserMsg + addedHorizontalRays +
          j * this->scanIterator->NumValidRaysInWholeScan();
      scan->mutable_ranges()->Set(index, range);
      scan->mutable_intensities()->Set(index, intensity);

      ++addedHorizontalRays;
    }
  }
  this->nextHorizontalPosInLaserMsg += addedHorizontalRays;
}

RotatingLidarSensor::~RotatingLidarSensor() // NOLINT(hicpp-use-equals-default)
{
}

bool RotatingLidarSensor::IsActive() const
{
  return Sensor::IsActive() ||
      (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

void RotatingLidarSensor::Reset()
{
  this->dataPtr->scanIterator->Reset();
  this->dataPtr->nextHorizontalPosInLaserMsg = 0;
}

common::Time RotatingLidarSensor::TimeIncrement() const
{
  return this->dataPtr->timeIncrement;
}

common::Time RotatingLidarSensor::ScanTime() const
{
  return this->dataPtr->scanTime;
}

}
}
#include "gtest/gtest.h"
#include <gazebo_rotating_lidar/ScanIterator.h>

#include <algorithm>
#include <gazebo/common/Exception.hh>

using namespace gazebo::sensors;
using ignition::math::Angle;
using gazebo::common::Time;

TEST(ScanIterator, Sick270Deg)
{
  const auto maxAngle = Angle::Pi * 3 / 4;
  const auto minAngle = Angle::Zero - maxAngle;
  const auto angleIncrement = Angle(0.00872664619237);

  ScanIterator it(minAngle, maxAngle, angleIncrement,
                  Time(2.77777780866e-05), Time(0.001));

  bool validScan;
  bool newScan;
  bool finishScan;
  size_t numUpdates;
  size_t numRays;
  size_t numValidRays;
  ScanRayFan fan;

  it.newScan = [&newScan](){ newScan = true; };
  it.firstValidScan = [&validScan](){ validScan = true; };
  it.finishScan = [&finishScan](){ finishScan = true; };
  it.updateScan = [&numUpdates, &fan, &numRays, &numValidRays](const ScanRayFan& scan) {
    ++numUpdates;
    numRays += scan.NumRays();
    numValidRays += scan.NumValidRays();
    fan = scan;
  };

  // the loop is here to make sure this scan is not affected by floating-point
  // computation imprecisions (because the number of rays per update is integral)
  for (size_t j = 0; j < 200; ++j)
  {
//    std::cout << "j = " << j << std::endl;

    newScan = false;
    validScan = false;
    finishScan = false;
    numRays = 0;
    numValidRays = 0;
    numUpdates = 0;

    it.Update();

    EXPECT_TRUE(newScan);
    EXPECT_FALSE(finishScan);
    EXPECT_TRUE(validScan);
    EXPECT_EQ(1, numUpdates);
    EXPECT_EQ(36, fan.NumRays());
    EXPECT_EQ(36, numRays);
    EXPECT_EQ(36, fan.NumValidRays());
    EXPECT_EQ(36, numValidRays);
    EXPECT_TRUE(fan.HasValidRays());
    EXPECT_TRUE(fan.AllRaysValid());
    EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
    EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 35).Radian(), fan.MaxAngle().Radian());

    for (size_t i = 1; i < 19; ++i)
    {
//      std::cout << "j = " << j << ", i = " << i << std::endl;
      newScan = false;
      validScan = false;
      finishScan = false;

      it.Update();

      EXPECT_FALSE(newScan);
      EXPECT_FALSE(finishScan);
      EXPECT_FALSE(validScan);
      EXPECT_EQ(i + 1, numUpdates);
      EXPECT_EQ(36, fan.NumRays());
      EXPECT_EQ((i + 1) * 36, numRays);
      EXPECT_EQ(std::min<size_t>((i + 1) * 36, 541), numValidRays);

      if (i < 15)
      {
        EXPECT_TRUE(fan.HasValidRays());
        EXPECT_TRUE(fan.AllRaysValid());
        EXPECT_EQ(36, fan.NumValidRays());
      }
      else if (i == 15)
      {
        EXPECT_TRUE(fan.HasValidRays());
        EXPECT_FALSE(fan.AllRaysValid());
        EXPECT_EQ(1, fan.NumValidRays());
      }
      else
      {
        EXPECT_FALSE(fan.HasValidRays());
        EXPECT_FALSE(fan.AllRaysValid());
        EXPECT_EQ(0, fan.NumValidRays());
      }
    }

    newScan = false;
    validScan = false;
    finishScan = false;

    it.Update();

    EXPECT_FALSE(newScan);
    EXPECT_TRUE(finishScan);
    EXPECT_FALSE(validScan);
    EXPECT_EQ(20, numUpdates);
    EXPECT_EQ(36, fan.NumRays());
    EXPECT_EQ(0, fan.NumValidRays());
    EXPECT_EQ(720, numRays);
    EXPECT_EQ(541, numValidRays);
    EXPECT_FALSE(fan.HasValidRays());
    EXPECT_FALSE(fan.AllRaysValid());

    // test that Reset() works
    if (j > 5)
    {
      for (size_t k = 0; k < j / 10; ++k)
      {
        it.Update();
      }
      it.Reset();
    }
  }
}

TEST(ScanIterator, Sick360Deg)
{
  const auto maxAngle = Angle::Pi;
  const auto minAngle = Angle::Zero - maxAngle;
  const auto angleIncrement = Angle(0.00872664619237);

  ScanIterator it(minAngle, maxAngle, angleIncrement,
                  Time(2.77777780866e-05), Time(0.001));

  bool validScan;
  bool newScan;
  bool finishScan;
  size_t numUpdates;
  size_t numRays;
  size_t numValidRays;
  ScanRayFan fan;

  it.newScan = [&newScan](){ newScan = true; };
  it.firstValidScan = [&validScan](){ validScan = true; };
  it.finishScan = [&finishScan](){ finishScan = true; };
  it.updateScan = [&numUpdates, &fan, &numRays, &numValidRays](const ScanRayFan& scan) {
    ++numUpdates;
    numRays += scan.NumRays();
    numValidRays += scan.NumValidRays();
    fan = scan;
  };

  // the loop is here to make sure this scan is not affected by floating-point
  // computation imprecisions (because the number of rays per update is integral)
  for (size_t j = 0; j < 200; ++j)
  {
    newScan = false;
    validScan = false;
    finishScan = false;
    numRays = 0;
    numValidRays = 0;
    numUpdates = 0;

    it.Update();

    EXPECT_TRUE(newScan);
    EXPECT_FALSE(finishScan);
    EXPECT_TRUE(validScan);
    EXPECT_EQ(1, numUpdates);
    EXPECT_EQ(36, fan.NumRays());
    EXPECT_EQ(36, numRays);
    EXPECT_EQ(36, fan.NumValidRays());
    EXPECT_EQ(36, numValidRays);
    EXPECT_TRUE(fan.HasValidRays());
    EXPECT_TRUE(fan.AllRaysValid());
    EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
    EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 35).Radian(), fan.MaxAngle().Radian());

    for (size_t i = 1; i < 19; ++i)
    {
      newScan = false;
      validScan = false;
      finishScan = false;

      it.Update();

      EXPECT_FALSE(newScan);
      EXPECT_FALSE(finishScan);
      EXPECT_FALSE(validScan);
      EXPECT_EQ(i + 1, numUpdates);
      EXPECT_EQ(36, fan.NumRays());
      EXPECT_EQ((i + 1) * 36, numRays);
      EXPECT_EQ((i + 1) * 36, numValidRays);

      EXPECT_TRUE(fan.HasValidRays());
      EXPECT_TRUE(fan.AllRaysValid());
      EXPECT_EQ(36, fan.NumValidRays());
    }

    newScan = false;
    validScan = false;
    finishScan = false;

    it.Update();

    EXPECT_FALSE(newScan);
    EXPECT_TRUE(finishScan);
    EXPECT_FALSE(validScan);
    EXPECT_EQ(20, numUpdates);
    EXPECT_EQ(36, fan.NumRays());
    EXPECT_EQ(36, fan.NumValidRays());
    EXPECT_EQ(720, numRays);
    EXPECT_EQ(720, numValidRays);
    EXPECT_TRUE(fan.HasValidRays());
    EXPECT_TRUE(fan.AllRaysValid());

    // test that Reset() works
    if (j > 5)
    {
      for (size_t k = 0; k < j / 10; ++k)
      {
        it.Update();
      }
      it.Reset();
    }
  }
}

TEST(ScanIterator, NonIntegralNumRaysPerUpdate)
{
  const auto maxAngle = Angle::HalfPi;
  const auto minAngle = Angle::Zero - maxAngle;
  const auto angleIncrement = Angle::Pi / 16;

  // 32 rays, first half of them valid

  ScanIterator it(minAngle, maxAngle, angleIncrement,
                  Time(0.3), Time(1.0));

  // we have 3.3333... rays per update

  bool validScan;
  bool newScan;
  bool finishScan;
  size_t numUpdates;
  size_t numRays;
  size_t numValidRays;
  ScanRayFan fan;

  it.newScan = [&newScan](){ newScan = true; };
  it.firstValidScan = [&validScan](){ validScan = true; };
  it.finishScan = [&finishScan](){ finishScan = true; };
  it.updateScan = [&numUpdates, &fan, &numRays, &numValidRays](const ScanRayFan& scan) {
    ++numUpdates;
    numRays += scan.NumRays();
    numValidRays += scan.NumValidRays();
    fan = scan;
  };

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = 0;
  numValidRays = 0;
  numUpdates = 0;

  it.Update(); // 3.33 rays

  EXPECT_TRUE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(1, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(3, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(3, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 2).Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 6.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(2, numUpdates);
  EXPECT_EQ(4, fan.NumRays());
  EXPECT_EQ(7, numRays);
  EXPECT_EQ(4, fan.NumValidRays());
  EXPECT_EQ(7, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 10.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(3, numUpdates);
  EXPECT_EQ(3, fan.NumRays());
  EXPECT_EQ(10, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(10, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 13.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(4, numUpdates);
  EXPECT_EQ(3, fan.NumRays());
  EXPECT_EQ(13, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(13, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 16.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(5, numUpdates);
  EXPECT_EQ(4, fan.NumRays());
  EXPECT_EQ(17, numRays);
  EXPECT_EQ(4, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 20.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(6, numUpdates);
  EXPECT_EQ(3, fan.NumRays());
  EXPECT_EQ(20, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 23.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(7, numUpdates);
  EXPECT_EQ(3, fan.NumRays());
  EXPECT_EQ(23, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 26.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(8, numUpdates);
  EXPECT_EQ(4, fan.NumRays());
  EXPECT_EQ(27, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 30.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(9, numUpdates);
  EXPECT_EQ(3, fan.NumRays());
  EXPECT_EQ(30, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());

  // now we wrap around 360 deg for the first time; we expect updateFunc to be
  // called twice - first with the remaining part of the old scan, and once
  // with the starting part of the new scan
  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 33.33 rays -> 1.33

  EXPECT_TRUE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(11, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(33, numRays);
  EXPECT_EQ(1, fan.NumValidRays());
  EXPECT_EQ(18, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());

  // Next update after wrapping around 360 deg. We've already had one valid ray
  // from this new scan in the previous update, so we don't trigger newScan nor
  // validScan.

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = fan.NumRays();
  numValidRays = fan.NumValidRays();

  it.Update(); // 4.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(12, numUpdates);
  ASSERT_EQ(4, fan.NumRays());
  EXPECT_EQ(5, numRays);
  EXPECT_EQ(4, fan.NumValidRays());
  EXPECT_EQ(5, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 4).Radian(), fan.MaxAngle().Radian());

  it.Update(); // 8 rays
  EXPECT_EQ(8, numValidRays);
  it.Update(); // 11.33 rays
  EXPECT_EQ(11, numValidRays);
  it.Update(); // 14.66 rays
  EXPECT_EQ(15, numValidRays);
  it.Update(); // 18 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 21.33 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 24.66 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 28 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 31.33 rays

  // again, looping over 360 degrees

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 34.66 rays -> 2.66

  EXPECT_TRUE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(22, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(35, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(20, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 2).Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = fan.NumRays();
  numValidRays = fan.NumValidRays();

  it.Update(); // 6.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(23, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(6, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(6, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 3).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 5).Radian(), fan.MaxAngle().Radian());

  it.Update(); // 9.33 rays
  EXPECT_EQ(9, numValidRays);
  it.Update(); // 12.66 rays
  EXPECT_EQ(13, numValidRays);
  it.Update(); // 16.00 rays
  EXPECT_EQ(16, numValidRays);
  it.Update(); // 19.33 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 22.66 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 26.00 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 29.33 rays
  EXPECT_EQ(17, numValidRays);

  // looping over 360 degrees

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 32.66 rays -> 0.66

  EXPECT_TRUE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(32, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(33, numRays);
  EXPECT_EQ(1, fan.NumValidRays());
  EXPECT_EQ(18, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = fan.NumRays();
  numValidRays = fan.NumValidRays();

  it.Update(); // 4.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(33, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(4, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(4, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 3).Radian(), fan.MaxAngle().Radian());

  it.Update(); // 7.33 rays
  EXPECT_EQ(7, numValidRays);
  it.Update(); // 10.66 rays
  EXPECT_EQ(11, numValidRays);
  it.Update(); // 14.00 rays
  EXPECT_EQ(14, numValidRays);
  it.Update(); // 17.33 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 20.66 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 24.00 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 27.33 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 30.66 rays
  EXPECT_EQ(17, numValidRays);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 34.00 rays -> 2.0

  EXPECT_TRUE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(43, numUpdates);
  ASSERT_EQ(2, fan.NumRays());
  EXPECT_EQ(34, numRays);
  EXPECT_EQ(2, fan.NumValidRays());
  EXPECT_EQ(19, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement).Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = fan.NumRays();
  numValidRays = fan.NumValidRays();

  it.Update(); // 5.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(44, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(5, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(5, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 2).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 4).Radian(), fan.MaxAngle().Radian());

  it.Update(); // 8.66 rays
  EXPECT_EQ(9, numValidRays);
  it.Update(); // 12.00 rays
  EXPECT_EQ(12, numValidRays);
  it.Update(); // 15.33 rays
  EXPECT_EQ(15, numValidRays);
  it.Update(); // 18.66 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 22.00 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 25.33 rays
  EXPECT_EQ(17, numValidRays);
  it.Update(); // 28.66 rays
  EXPECT_EQ(17, numValidRays);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 32.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(52, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(32, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(17, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 31).Radian(), fan.MaxAngle().Radian());

  // now we've reached a point where the scan finally aligns to the update rate
  // again and has no fractional part left on its beginning

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = 0;
  numValidRays = 0;

  it.Update(); // 3.33 rays

  EXPECT_TRUE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(53, numUpdates);
  ASSERT_EQ(3, fan.NumRays());
  EXPECT_EQ(3, numRays);
  EXPECT_EQ(3, fan.NumValidRays());
  EXPECT_EQ(3, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 2).Radian(), fan.MaxAngle().Radian());
}

TEST(ScanIterator, TimeIncrementLessThanUpdatePeriod)
{
  const auto maxAngle = Angle::HalfPi;
  const auto minAngle = Angle::Zero - maxAngle;
  const auto angleIncrement = Angle::Pi / 4;

  // 8 rays, first half of them valid

  ScanIterator it(minAngle, maxAngle, angleIncrement,
                  Time(0.3), Time(0.1));

  // we have 1/3 rays per update

  bool validScan;
  bool newScan;
  bool finishScan;
  size_t numUpdates;
  size_t numRays;
  size_t numValidRays;
  ScanRayFan fan;

  it.newScan = [&newScan]()
  { newScan = true; };
  it.firstValidScan = [&validScan]()
  { validScan = true; };
  it.finishScan = [&finishScan]()
  { finishScan = true; };
  it.updateScan = [&numUpdates, &fan, &numRays, &numValidRays](const ScanRayFan &scan)
  {
    ++numUpdates;
    numRays += scan.NumRays();
    numValidRays += scan.NumValidRays();
    fan = scan;
  };

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = 0;
  numValidRays = 0;
  numUpdates = 0;

  it.Update(); // 0.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(0, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 0.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(0, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 1.00 ray

  EXPECT_TRUE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(1, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(1, numRays);
  EXPECT_EQ(1, fan.NumValidRays());
  EXPECT_EQ(1, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 1.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(1, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 1.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(1, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 2.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(2, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(2, numRays);
  EXPECT_EQ(1, fan.NumValidRays());
  EXPECT_EQ(2, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement).Radian(), fan.MaxAngle().Radian());

  it.Update(); // 2.33 rays
  it.Update(); // 2.66 rays
  it.Update(); // 3.00 rays
  EXPECT_EQ(3, numValidRays);
  it.Update(); // 3.33 rays
  it.Update(); // 3.66 rays
  it.Update(); // 4.00 rays
  EXPECT_EQ(4, numValidRays);
  it.Update(); // 4.33 rays
  it.Update(); // 4.66 rays
  it.Update(); // 5.00 rays
  EXPECT_EQ(5, numValidRays);
  it.Update(); // 5.33 rays
  it.Update(); // 5.66 rays
  it.Update(); // 6.00 rays
  EXPECT_EQ(5, numValidRays);
  it.Update(); // 6.33 rays
  it.Update(); // 6.66 rays
  it.Update(); // 7.00 rays
  EXPECT_EQ(5, numValidRays);
  it.Update(); // 7.33 rays
  it.Update(); // 7.66 rays

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 8.00 rays

  EXPECT_FALSE(newScan);
  EXPECT_TRUE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(8, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(8, numRays);
  EXPECT_EQ(0, fan.NumValidRays());
  EXPECT_EQ(5, numValidRays);
  EXPECT_FALSE(fan.HasValidRays());
  EXPECT_FALSE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 7).Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ((minAngle + angleIncrement * 7).Radian(), fan.MaxAngle().Radian());

  newScan = false;
  validScan = false;
  finishScan = false;
  numRays = 0;
  numValidRays = 0;

  it.Update(); // 8.33 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(8, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 8.66 rays

  EXPECT_FALSE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_FALSE(validScan);
  EXPECT_EQ(8, numUpdates);

  newScan = false;
  validScan = false;
  finishScan = false;

  it.Update(); // 1.00 ray

  EXPECT_TRUE(newScan);
  EXPECT_FALSE(finishScan);
  EXPECT_TRUE(validScan);
  EXPECT_EQ(9, numUpdates);
  ASSERT_EQ(1, fan.NumRays());
  EXPECT_EQ(1, numRays);
  EXPECT_EQ(1, fan.NumValidRays());
  EXPECT_EQ(1, numValidRays);
  EXPECT_TRUE(fan.HasValidRays());
  EXPECT_TRUE(fan.AllRaysValid());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MinAngle().Radian());
  EXPECT_DOUBLE_EQ(minAngle.Radian(), fan.MaxAngle().Radian());
}

TEST(ScanIterator, InvalidConfig)
{
  // more scans per sensor update are not supported
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(1),
                  Time(0.1), Time(1.0)), gazebo::common::Exception);

  // min angle < max angle
  EXPECT_THROW(ScanIterator it(Angle(1), Angle(-1), Angle(1),
                  Time(1.0), Time(1.0)), gazebo::common::Exception);

  // angle increment > 0
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(0),
                  Time(1.0), Time(1.0)), gazebo::common::Exception);
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(-1),
                  Time(1.0), Time(1.0)), gazebo::common::Exception);

  // time increment must be positive
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(1),
                  Time(0.0), Time(1.0)), gazebo::common::Exception);
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(1),
                  Time(-1.0), Time(1.0)), gazebo::common::Exception);

  // update rate must be positive
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(1),
                  Time(1.0), Time(0.0)), gazebo::common::Exception);
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(1),
                  Time(1.0), Time(-1.0)), gazebo::common::Exception);

  // angular range must be a multiple of angle increment
  EXPECT_THROW(ScanIterator it(Angle(-1), Angle(1), Angle(0.3),
                  Time(1.0), Time(1.0)), gazebo::common::Exception);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
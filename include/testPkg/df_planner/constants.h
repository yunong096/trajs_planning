#pragma once
namespace plan_manage {
static constexpr double kEpsilon = 1.0e-6;
static constexpr double epis = 1e-6;
// corridor generation
static constexpr double kLatExtendStep = 0.1;
static constexpr double kLonExtendStep = 0.2;
static constexpr double kCorridorWidthBase = 2.5;
static constexpr double kCorridorWidthExtendRatio = 0.15;
static constexpr double kBoxExtendStep = 2.0;
static constexpr double kLimitBound = 15.0;

}  // namespace plan_manage

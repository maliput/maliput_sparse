// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "geometry/lane_geometry.h"

#include "maliput_sparse/geometry/utility/geometry.h"

namespace maliput_sparse {
namespace geometry {

namespace {}  // namespace

LaneGeometry::LaneGeometry(const LineString3d& left, const LineString3d& right, double linear_tolerance,
                           double scale_length)
    : left_(left),
      right_(right),
      linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      centerline_(utility::ComputeCenterline3d(left_, right_)) {}

double LaneGeometry::ArcLength() const { return centerline_.length(); }

maliput::math::Vector3 LaneGeometry::W(const maliput::math::Vector3& prh) const {
  // Obtains the point on the centerline (p, 0, 0).
  const maliput::math::Vector3 on_centerline_point = utility::InterpolatedPointAtP(centerline_, prh.x());
  // Calculates orientation of (p,r,h) basis at (p,0,0).
  const maliput::math::RollPitchYaw rpy = Orientation(prh.x());
  // Rotates (0,r,h) and sums with mapped (p,0,0).
  return rpy.ToMatrix() * maliput::math::Vector3(0., prh.y(), prh.z()) + on_centerline_point;
}

maliput::math::Vector3 LaneGeometry::WDot(double p) const { return utility::GetTangentAtP(centerline_, p); };

maliput::math::Vector3 LaneGeometry::WDot(const maliput::math::Vector3& prh) const {
  // Implementation based on
  // https://github.com/maliput/maliput_malidrive/blob/2eb648a030fa4439370b6ed757b84cb726459896/src/maliput_malidrive/road_curve/road_curve.cc#L92-L122
  MALIPUT_THROW_UNLESS(prh.x() >= p0());
  MALIPUT_THROW_UNLESS(prh.x() <= p1());
  const double p = prh.x();
  const double r = prh.y();
  const double h = prh.z();

  const maliput::math::RollPitchYaw rpy_at_centerline = Orientation(p);

  // Evaluate dα/dp, dβ/dp, dγ/dp...

  // TODO(francocipollone): Once superelevation is computed (roll) d_alpha should be updated to be the derivative of
  // the superelevation at p
  const double d_alpha = 0.;
  // For computing dβ/dp we need the second derivative of the elevation.
  // As the lane is linearly changing elevation, the second derivative is zero.
  const double d_beta = 0.;
  // For computing dγ/dp we need the second derivative of the heading.
  // As the lane is linearly changing heading, the second derivative is zero.
  const double d_gamma = 0.;

  // compute rotation matrix at centerline (R) and its time derivative (dR_dt)
  const maliput::math::Matrix3 R = rpy_at_centerline.ToMatrix();
  const maliput::math::Matrix3 dR_dt = rpy_at_centerline.CalcRotationMatrixDt({d_alpha, d_beta, d_gamma});

  // TODO(francocipollone): Compute dr/dp correctly. We need to account for lane offset derivatives.
  const double r_dot = 0.;
  return WDot(p) + dR_dt * maliput::math::Vector3(0, r, h) + R * maliput::math::Vector3{0., r_dot, 0.};
}

maliput::math::RollPitchYaw LaneGeometry::Orientation(double p) const {
  const double superelevation{0.}; /* TODO: Take superelevation into account */
  return maliput::math::RollPitchYaw(
      superelevation,
      -std::atan2(utility::GetSlopeAtP(centerline_, p), utility::Get2DTangentAtP(centerline_, p).norm()),
      utility::Get2DHeadingAtP(centerline_, p));
}

maliput::math::RollPitchYaw LaneGeometry::Orientation(const maliput::math::Vector3& prh) const {
  // TODO: Take into account r and h displacement.
  return Orientation(prh.x());
}

maliput::math::Vector3 LaneGeometry::WInverse(const maliput::math::Vector3& xyz) const {
  const utility::ClosestPointResult closest_centerline_point = utility::GetClosestPoint(centerline_, xyz);
  const double p = closest_centerline_point.p;

  const maliput::math::Vector3 d_xyz = xyz - closest_centerline_point.point;

  const maliput::math::Matrix3 rotation = Orientation(closest_centerline_point.p).ToMatrix();
  const maliput::math::Vector3 r_hat = rotation * maliput::math::Vector3::UnitY();
  const maliput::math::Vector3 h_hat = rotation * maliput::math::Vector3::UnitZ();

  const double r = d_xyz.dot(r_hat);
  const double h = d_xyz.dot(h_hat);
  return {p, r, h};
}

maliput::api::RBounds LaneGeometry::RBounds(double p) const {
  MALIPUT_THROW_UNLESS(p >= p0());
  MALIPUT_THROW_UNLESS(p <= p1());

  // Get p equivalent left and right.
  // Get distance from centerline to left and right.
  const double p_left = FromCenterPToLateralP(LineStringType::kLeftBoundary, p);
  const double p_right = FromCenterPToLateralP(LineStringType::kRightBoundary, p);

  const maliput::math::Vector3 on_centerline = utility::InterpolatedPointAtP(centerline_, p);
  const maliput::math::Vector3 on_left = utility::InterpolatedPointAtP(left_, p_left);
  const maliput::math::Vector3 on_right = utility::InterpolatedPointAtP(right_, p_right);
  return {-(on_right - on_centerline).norm(), (on_left - on_centerline).norm()};
}

double LaneGeometry::FromCenterPToLateralP(const LineStringType& line_string_type, double p) const {
  MALIPUT_THROW_UNLESS(line_string_type != LineStringType::kCenterLine);
  const double lateral_length = line_string_type == LineStringType::kLeftBoundary ? left_.length() : right_.length();
  const double p_equivalent = p * lateral_length / centerline_.length();
  return std::clamp(p_equivalent, 0., lateral_length);
}

}  // namespace geometry
}  // namespace maliput_sparse

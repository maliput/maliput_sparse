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

LaneGeometry::LaneGeometry(const LineString3d& left, const LineString3d& right, double scale_length,
                           double linear_tolerance)
    : left_(left),
      right_(right),
      scale_length_(scale_length),
      linear_tolerance_(linear_tolerance),
      centerline_(utility::ComputeCenterline3d(left_, right_)) {}

double LaneGeometry::ArcLength() { return centerline_.length(); }

maliput::math::Vector3 LaneGeometry::W(const maliput::math::Vector3& prh) const {
  // Obtains the point on the centerline (p, 0, 0).
  const maliput::math::Vector3 on_centerline_point = utility::InterpolatedPointAtP(centerline_, prh.x());
  // Calculates orientation of (p,r,h) basis at (p,0,0).
  const maliput::math::RollPitchYaw rpy = Orientation(prh.x());
  // Rotates (0,r,h) and sums with mapped (p,0,0).
  return rpy.ToMatrix() * maliput::math::Vector3(0., prh.y(), prh.z()) + on_centerline_point;
}

maliput::math::Vector3 LaneGeometry::WDot(const maliput::math::Vector3& prh) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::WDot is not implemented yet.");
}

maliput::math::RollPitchYaw LaneGeometry::Orientation(double p) const {
  const double superelevation{0.}; /* TODO: Take superelevation into account */
  return maliput::math::RollPitchYaw(
      superelevation,
      -std::atan2(utility::GetSlopeAtP(centerline_, p), utility::Get2DTangentAtP(centerline_, p).norm()),
      utility::Get2DHeadingAtP(centerline_, p));
}

maliput::math::RollPitchYaw LaneGeometry::Orientation(const maliput::math::Vector3& prh) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::Orientation(prh) is not implemented yet.");
}

maliput::math::Vector3 LaneGeometry::WInverse(const maliput::math::Vector3& xyz) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::WInverse is not implemented yet.");
}

}  // namespace geometry
}  // namespace maliput_sparse

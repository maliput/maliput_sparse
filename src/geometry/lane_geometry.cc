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
#include "lane_geometry.h"


namespace maliput_sample {
namespace geometry {

namespace {

// TODO(): Move this method to a common place for all geometry helper methods.
//
// Returns the piecewise linearly interpolated point at the given distance.
// @param lineString the lineString to iterate. Size must be >0.
// @param p distance along linestring.
// @return The interpolated point (a new point if not perfectly matching)
//
maliput::math::Vector3 InterpolatedPointAtP(const LineString<maliput::math::Vector3>& line_string, double p) {
  MALIPUT_THROW_MESSAGE("Not implemented yet.");
  // TODO() Move method to a common place for all geometry helper methods.
}

LineString<maliput::math::Vector3> ComputeCenterline(const LineString<maliput::math::Vector3>& left, const LineString<maliput::math::Vector3>& right) {
  MALIPUT_THROW_MESSAGE("Not implemented yet.");
  // TODO() Move method to a common place for all geometry helper methods.
}

maliput::math::Vector3 GetPHat(const LineString<maliput::math::Vector3>& line_string, double p) {
  MALIPUT_THROW_MESSAGE("Not implemented yet.");
  // TODO() Move method to a common place for all geometry helper methods.
}


}  // namespace

LaneGeometry::LaneGeometry(const LineString<maliput::math::Vector3>& left,
                           const LineString<maliput::math::Vector3>& right, double scale_length,
                           double linear_tolerance)
    : left_(left), right_(right), scale_length_(scale_length), linear_tolerance_(linear_tolerance) {
  centerline_ = ComputeCenterline(left_, right_);
}

maliput::math::Vector3 LaneGeometry::W(const maliput::math::Vector3& prh) const {
  const double p = prh.x();
  const double r = prh.y();
  const double h = prh.z();
  // Obtains the point on the centerline (p, 0, 0).
  const maliput::math::Vector3 on_centerline_point = InterpolatedPointAtP(centerline_, p);
  // Obtains r_hat.
  const maliput::math::Vector3 on_left_point = InterpolatedPointAtP(left_, p * left_.length() / centerline_.length());
  const maliput::math::Vector3 on_right_point =
      InterpolatedPointAtP(right_, p * right_.length() / centerline_.length());
  const maliput::math::Vector3 r_direction = on_right_point - on_left_point;
  const maliput::math::Vector3 r_hat = r_direction.normalized();

  // Obtains h_hat.
  // p_hat x r_hat = h_hat
  // To obtain p_hat we can start from the p_hat of the boundaries and scale it to right r value.
  const maliput::math::Vector3 left_p_hat = GetPHat(left_, p);
  const maliput::math::Vector3 right_p_hat = GetPHat(right_, p);

  const double distance_left_right = (on_right_point - on_left_point).norm();
  const double factor = (distance_left_right / 2. + r) / distance_left_right;
  // Gets p_hat on prh point.
  const maliput::math::Vector3 p_hat = right_p_hat * factor + left_p_hat * (1. - factor);
  const maliput::math::Vector3 h_hat = r_hat.cross(p_hat);

  // Computes the final point
  const maliput::math::Vector3 on_prh_point = on_centerline_point + r_hat * r + h_hat * h;
  return on_prh_point;
}

maliput::math::Vector3 LaneGeometry::WDot(const maliput::math::Vector3& prh) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::WDot is not implemented yet.");


  // Same direction as SHat but with a module, how to know the module?


}

maliput::math::RollPitchYaw LaneGeometry::Orientation(const maliput::math::Vector3& prh) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::Orientation is not implemented yet.");

  // Knowing s_hat, r_hat and h_hat we can compute the orientation.

}

maliput::math::Vector3 LaneGeometry::WInverse(const maliput::math::Vector3& xyz) const {
  MALIPUT_THROW_MESSAGE("LaneGeometry::Orientation is not implemented yet.");

  // Halp!
}

}  // namespace geometry
}  // namespace maliput_sample

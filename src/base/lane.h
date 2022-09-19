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
#pragma once

#include <memory>

#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/geometry_base/lane.h>
#include <maliput/math/vector.h>

#include "maliput_sparse/geometry/lane_geometry.h"

namespace maliput_sparse {

class Lane : public maliput::geometry_base::Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Lane);
  /// Constructs a Lane
  /// @param id The Lane's unique identifier.
  /// @param elevation_bounds The Lane's elevation bounds.
  /// @param lane_geometry A LaneGeometry.
  Lane(const maliput::api::LaneId& id, const maliput::api::HBounds& elevation_bounds,
       std::unique_ptr<geometry::LaneGeometry> lane_geometry);

  maliput::math::Vector3 ToBackendPosition(const maliput::api::LanePosition& lane_pos) const {
    return DoToBackendPosition(lane_pos);
  }

  const geometry::LaneGeometry* lane_geometry() const { return lane_geometry_.get(); }

 private:
  // maliput::api::Lane private virtual method implementations.
  //@{
  double do_length() const override;
  maliput::api::RBounds do_lane_bounds(double s) const override;
  maliput::api::RBounds do_segment_bounds(double s) const override;
  maliput::api::HBounds do_elevation_bounds(double, double) const override;

  maliput::math::Vector3 DoToBackendPosition(const maliput::api::LanePosition& lane_pos) const override;
  void DoToLanePositionBackend(const maliput::math::Vector3& backend_pos, maliput::api::LanePosition* lane_position,
                               maliput::math::Vector3* nearest_backend_pos, double* distance) const override;
  maliput::api::Rotation DoGetOrientation(const maliput::api::LanePosition& lane_pos) const override;
  maliput::api::LanePosition DoEvalMotionDerivatives(const maliput::api::LanePosition& position,
                                                     const maliput::api::IsoLaneVelocity& velocity) const override;
  //@}

  const maliput::api::HBounds elevation_bounds_;
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_;
};

}  // namespace maliput_sparse

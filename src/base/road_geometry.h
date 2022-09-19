// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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

#include <optional>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/geometry_base/road_geometry.h>
#include <maliput/math/vector.h>

namespace maliput_sparse {

class RoadGeometry final : public maliput::geometry_base::RoadGeometry {
 public:
  RoadGeometry(const maliput::api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance,
               double scale_length, const maliput::math::Vector3& inertial_to_backend_frame_translation)
      : maliput::geometry_base::RoadGeometry(id, linear_tolerance, angular_tolerance, scale_length,
                                             inertial_to_backend_frame_translation) {}

 private:
  maliput::api::RoadPositionResult DoToRoadPosition(
      const maliput::api::InertialPosition& inertial_position,
      const std::optional<maliput::api::RoadPosition>& hint = std::nullopt) const override {
    return maliput::api::RoadPositionResult();
  }

  std::vector<maliput::api::RoadPositionResult> DoFindRoadPositions(
      const maliput::api::InertialPosition& inertial_position, double radius) const {
    return std::vector<maliput::api::RoadPositionResult>();
  }
};

}  // namespace maliput_sparse

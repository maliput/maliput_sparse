// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <maliput/api/lane_boundary.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/geometry_base/lane_boundary.h>

#include "maliput_sparse/parser/lane_marking.h"

namespace maliput_sparse {

class LaneBoundary final : public maliput::geometry_base::LaneBoundary {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneBoundary)

  /// Constructs a LaneBoundary for the sparse backend.
  ///
  /// @param id The unique identifier for this boundary.
  /// @param lane_to_left The lane on the +r side, or nullptr for the leftmost boundary.
  /// @param lane_to_right The lane on the -r side, or nullptr for the rightmost boundary.
  /// @param markings Optional marking data for this boundary, sorted by s_start.
  ///
  /// @throws maliput::common::assertion_error if both lane pointers are nullptr.
  LaneBoundary(const maliput::api::LaneBoundary::Id& id, const maliput::api::Lane* lane_to_left,
               const maliput::api::Lane* lane_to_right, const std::vector<parser::BoundaryMarkings>& markings = {});

  ~LaneBoundary() override = default;

 private:
  const maliput::api::Lane* do_lane_to_left() const override { return lane_to_left_; }
  const maliput::api::Lane* do_lane_to_right() const override { return lane_to_right_; }
  std::optional<maliput::api::LaneMarkingResult> DoGetMarking(double s) const override;
  std::vector<maliput::api::LaneMarkingResult> DoGetMarkings() const override;
  std::vector<maliput::api::LaneMarkingResult> DoGetMarkings(double s_start, double s_end) const override;

  /// Helper structure to store marking data.
  struct MarkingData {
    double s_start{0.};
    double s_end{0.};
    parser::LaneMarking marking{};
  };

  const maliput::api::Lane* lane_to_left_{};
  const maliput::api::Lane* lane_to_right_{};
  std::vector<MarkingData> markings_{};
};

}  // namespace maliput_sparse

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

#include "base/lane_boundary.h"

#include <algorithm>

#include <maliput/api/lane_marking.h>
#include <maliput/common/maliput_throw.h>

namespace maliput_sparse {
namespace {

// Converts a parser::LaneMarkingLine to maliput::api::LaneMarkingLine.
maliput::api::LaneMarkingLine ToApiMarkingLine(const parser::LaneMarkingLine& line) {
  return maliput::api::LaneMarkingLine{line.length, line.space, line.width, line.r_offset, line.color};
}

// Converts a parser::LaneMarking to maliput::api::LaneMarking.
maliput::api::LaneMarking ToApiMarking(const parser::LaneMarking& marking) {
  maliput::api::LaneMarking result{};
  result.type = marking.type;
  result.weight = marking.weight;
  result.color = marking.color;
  result.width = marking.width.value_or(0.);
  result.height = marking.height.value_or(0.);
  result.lane_change = marking.lane_change;
  result.material = marking.material;
  result.lines.reserve(marking.lines.size());
  for (const auto& line : marking.lines) {
    result.lines.push_back(ToApiMarkingLine(line));
  }
  return result;
}

// Converts a LaneBoundary::MarkingData to maliput::api::LaneMarkingResult.
maliput::api::LaneMarkingResult ToApiMarkingResultFromParts(double s_start, double s_end,
                                                            const parser::LaneMarking& marking) {
  return maliput::api::LaneMarkingResult{ToApiMarking(marking), s_start, s_end};
}

}  // namespace

LaneBoundary::LaneBoundary(const maliput::api::LaneBoundary::Id& id, const maliput::api::Lane* lane_to_left,
                           const maliput::api::Lane* lane_to_right,
                           const std::vector<parser::BoundaryMarkings>& markings)
    : maliput::geometry_base::LaneBoundary(id), lane_to_left_(lane_to_left), lane_to_right_(lane_to_right) {
  MALIPUT_THROW_UNLESS(lane_to_left_ != nullptr || lane_to_right_ != nullptr);
  // Convert BoundaryMarkings to internal MarkingData format and store.
  markings_.reserve(markings.size());
  for (const auto& marking : markings) {
    markings_.push_back(MarkingData{marking.s_start, marking.s_end, marking.marking});
  }
  // Ensure markings are sorted by s_start.
  std::sort(markings_.begin(), markings_.end(),
            [](const MarkingData& a, const MarkingData& b) { return a.s_start < b.s_start; });
}

std::optional<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarking(double s) const {
  // Binary search for the first marking whose s_end > s (i.e., not fully before s).
  auto it = std::lower_bound(markings_.begin(), markings_.end(), s,
                             [](const MarkingData& marking, double value) { return marking.s_end <= value; });

  if (it != markings_.end() && it->s_start <= s) {
    return ToApiMarkingResultFromParts(it->s_start, it->s_end, it->marking);
  }
  return std::nullopt;
}

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings() const {
  std::vector<maliput::api::LaneMarkingResult> results;
  results.reserve(markings_.size());
  for (const auto& marking : markings_) {
    results.push_back(ToApiMarkingResultFromParts(marking.s_start, marking.s_end, marking.marking));
  }
  return results;
}

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings(double s_start, double s_end) const {
  std::vector<maliput::api::LaneMarkingResult> results;
  for (const auto& marking : markings_) {
    // Include markings that overlap with [s_start, s_end).
    if (marking.s_end > s_start && marking.s_start < s_end) {
      results.push_back(ToApiMarkingResultFromParts(marking.s_start, marking.s_end, marking.marking));
    }
  }
  return results;
}

}  // namespace maliput_sparse

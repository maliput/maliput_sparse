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

#include <maliput/common/maliput_throw.h>

namespace maliput_sparse {

LaneBoundary::LaneBoundary(const maliput::api::LaneBoundary::Id& id, const maliput::api::Lane* lane_to_left,
                           const maliput::api::Lane* lane_to_right)
    : maliput::geometry_base::LaneBoundary(id), lane_to_left_(lane_to_left), lane_to_right_(lane_to_right) {
  MALIPUT_THROW_UNLESS(lane_to_left_ != nullptr || lane_to_right_ != nullptr);
}

std::optional<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarking(double) const { return std::nullopt; }

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings() const { return {}; }

std::vector<maliput::api::LaneMarkingResult> LaneBoundary::DoGetMarkings(double, double) const { return {}; }

}  // namespace maliput_sparse

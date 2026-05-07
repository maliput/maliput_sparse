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
#include <string>
#include <vector>

#include <maliput/api/lane_marking.h>

namespace maliput_sparse {
namespace parser {

/// @brief Represents a single line in a lane marking (for complex markings with multiple lines).
struct LaneMarkingLine {
  /// Length of the visible (painted) part of each dash, in meters.
  /// For solid lines, this should be 0 (value is ignored).
  double length{0.};

  /// Length of the gap between visible parts, in meters.
  /// For solid lines, this should be 0.
  double space{0.};

  /// Width of this line, in meters.
  double width{0.};

  /// Lateral offset from the lane boundary, in meters.
  /// Positive values offset in the positive r-direction (towards the lane's left).
  double r_offset{0.};

  /// Color of this line.
  maliput::api::LaneMarkingColor color{maliput::api::LaneMarkingColor::kUnknown};

  /// Equality operator.
  bool operator==(const LaneMarkingLine& other) const {
    return length == other.length && space == other.space && width == other.width && r_offset == other.r_offset &&
           color == other.color;
  }

  /// Inequality operator.
  bool operator!=(const LaneMarkingLine& other) const { return !(*this == other); }
};

/// @brief Represents the complete marking at a lane boundary for a specific s-range.
struct LaneMarking {
  /// Type of marking (solid, dashed, etc.).
  maliput::api::LaneMarkingType type{maliput::api::LaneMarkingType::kUnknown};

  /// Color of the marking.
  maliput::api::LaneMarkingColor color{maliput::api::LaneMarkingColor::kUnknown};

  /// Visual weight of the marking (standard, bold, etc.).
  maliput::api::LaneMarkingWeight weight{maliput::api::LaneMarkingWeight::kUnknown};

  /// Total width of the marking, in meters (optional).
  std::optional<double> width{};

  /// Height of the marking, in meters (optional, for raised markings).
  std::optional<double> height{};

  /// Material of the marking (optional).
  std::string material{};

  /// Lane change permission this marking allows.
  maliput::api::LaneChangePermission lane_change{maliput::api::LaneChangePermission::kUnknown};

  /// Detailed line definitions for complex markings (e.g., double lines).
  /// For simple markings, this can be empty.
  std::vector<LaneMarkingLine> lines{};

  /// Equality operator.
  bool operator==(const LaneMarking& other) const {
    return type == other.type && color == other.color && weight == other.weight && width == other.width &&
           height == other.height && material == other.material && lane_change == other.lane_change &&
           lines == other.lines;
  }

  /// Inequality operator.
  bool operator!=(const LaneMarking& other) const { return !(*this == other); }
};

/// @brief Represents a marking and the s-range over which it is valid on a lane boundary.
struct BoundaryMarkings {
  /// Start s-coordinate where the marking begins.
  double s_start{0.};

  /// End s-coordinate where the marking ends.
  double s_end{0.};

  /// The marking data.
  LaneMarking marking{};

  /// Equality operator.
  bool operator==(const BoundaryMarkings& other) const {
    return s_start == other.s_start && s_end == other.s_end && marking == other.marking;
  }

  /// Inequality operator.
  bool operator!=(const BoundaryMarkings& other) const { return !(*this == other); }
};

}  // namespace parser
}  // namespace maliput_sparse

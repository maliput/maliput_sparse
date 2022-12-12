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
#include "maliput_sparse/parser/validator.h"

#include "maliput_sparse/geometry/utility/geometry.h"
#include "maliput_sparse/parser/junction.h"
#include "maliput_sparse/parser/lane.h"
#include "maliput_sparse/parser/segment.h"

namespace maliput_sparse {
namespace parser {

namespace {

// Convenient constant for indicating left or right.
static constexpr bool kLeft{true};
static constexpr bool kRight{!kLeft};

}  // namespace

Validator::Validator(const Parser* parser, const ValidatorOptions& options, const ValidatorConfig config) {
  MALIPUT_THROW_UNLESS(parser);
  if (options.lane_adjacency) {
    ValidateLaneAdjacency(parser, config);
  }
}

const std::vector<Validator::Error>& Validator::GetErrors() const { return errors_; }

void Validator::CheckAdjacency(const Lane& lane, const Lane& adjacent_lane, bool left, double tolerance) {
  const auto line_string_a = left ? lane.left : lane.right;
  const auto line_string_b = left ? adjacent_lane.right : adjacent_lane.left;
  const double distance = geometry::utility::ComputeDistance(line_string_a, line_string_b);
  if (distance > tolerance) {
    const std::string msg{"Lane " + lane.id + " and lane " + adjacent_lane.id +
                          " are not adjacent under the tolerance " + std::to_string(tolerance) +
                          ". (distance: " + std::to_string(distance) + ")"};
    Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
  }
}

void Validator::Report(const std::string& message, const Validator::Error::Type& type,
                       const Validator::Error::Severity& severity) {
  errors_.push_back(Validator::Error{message, type, severity});
}

void Validator::ValidateLaneAdjacency(const Parser* parser, const ValidatorConfig config) {
  const auto junctions = parser->GetJunctions();
  for (const auto junction : junctions) {
    for (const auto segment : junction.second.segments) {
      const std::vector<Lane>& lanes = segment.second.lanes;
      // Iterates lanes from right to left.
      // | idx=N-1 | idx=3 | idx=2 | idx=1 | idx=0 |
      for (int idx{}; idx < static_cast<int>(lanes.size()); ++idx) {
        const Lane& lane = lanes[idx];

        // Note: The following code is a bit repetitive for left and right adjacency checks, but it is easier to read
        // and understand. Check right adjacency <--------------------------------------------------
        if (lane.right_lane_id) {
          // Check if there is a idx to the right.
          if (idx == 0) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                                  lane.right_lane_id.value() + ") but is the first lane in the segment " +
                                  segment.first + "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }
          // Check if right lane id is in the same segment
          const auto adj_lane_it = std::find_if(lanes.begin(), lanes.end(), [&lane](const Lane& lane_it) {
            return lane.right_lane_id.value() == lane_it.id;
          });
          if (adj_lane_it == lanes.end()) {
            // Error.
            const std::string msg{"Adjacent lane isn't part of the segment: Lane " + lane.id +
                                  " has a right lane id (" + lane.right_lane_id.value() +
                                  ") that is not part of the segment " + segment.first + "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          } else {
            // Check if right lane id has the lane id as left lane id.
            if (adj_lane_it->left_lane_id) {
              if (adj_lane_it->left_lane_id.value() != lane.id) {
                // Error.
                const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                                      lane.right_lane_id.value() + ") that has a left lane id (" +
                                      adj_lane_it->left_lane_id.value() + ") that is not the lane " + lane.id + "."};
                Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
              }
            } else {
              // Error.
              const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                                    lane.right_lane_id.value() + ") that has no left lane id."};
              Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
            }

            // Check geometrical adjacency.
            CheckAdjacency(lane, *adj_lane_it, kRight, config.linear_tolerance);
          }

          // Check if idx - 1 lane is the right lane id.
          if ((idx - 1 >= 0) && (lanes[idx - 1].id != lane.right_lane_id.value())) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a right lane id (" +
                                  lane.right_lane_id.value() + ") that is not the previous lane in the segment " +
                                  segment.first + "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }

        } else {
          // Check if idx is the first lane in the segment.
          if (idx != 0) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id +
                                  " has no right lane id but it isn't the first lane in the segment " + segment.first +
                                  "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }
        }

        // Check left adjacency <--------------------------------------------------
        if (lane.left_lane_id) {
          // Check if there is a idx to the left.
          if (idx == static_cast<int>(lanes.size()) - 1) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                                  lane.left_lane_id.value() + ") but is the last lane in the segment " + segment.first +
                                  "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }
          // Check if left lane id is in the same segment
          const auto adj_lane_it = std::find_if(lanes.begin(), lanes.end(), [&lane](const Lane& lane_it) {
            return lane.left_lane_id.value() == lane_it.id;
          });
          if (adj_lane_it == lanes.end()) {
            // Error.
            const std::string msg{"Adjacent lane isn't part of the segment: Lane " + lane.id + " has a left lane id (" +
                                  lane.left_lane_id.value() + ") that is not part of the segment " + segment.first +
                                  "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          } else {
            // Check if left lane id has the lane id as right lane id.
            if (adj_lane_it->right_lane_id) {
              if (adj_lane_it->right_lane_id.value() != lane.id) {
                // Error.
                const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                                      lane.left_lane_id.value() + ") that has a right lane id (" +
                                      adj_lane_it->right_lane_id.value() + ") that is not the lane " + lane.id + "."};
                Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
              }
            } else {
              // Error.
              const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                                    lane.left_lane_id.value() + ") that has no right lane id."};
              Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
            }

            // Check geometrical adjacency.
            CheckAdjacency(lane, *adj_lane_it, kLeft, config.linear_tolerance);
          }

          // Check if idx + 1 lane is the left lane id.
          if ((idx + 1 <= static_cast<int>(lanes.size()) - 1) && (lanes[idx + 1].id != lane.left_lane_id.value())) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id + " has a left lane id (" +
                                  lane.left_lane_id.value() + ") that is not the next lane in the segment " +
                                  segment.first + "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }

        } else {
          // Check if idx is the last lane in the segment.
          if (idx != static_cast<int>(lanes.size()) - 1) {
            // Error.
            const std::string msg{"Wrong ordering of lanes: Lane " + lane.id +
                                  " has no left lane id but it isn't the last lane in the segment " + segment.first +
                                  "."};
            Report(msg, Validator::Error::Type::kLaneAdjacency, Validator::Error::Severity::kError);
          }
        }
      }
    }
  }
}

bool Validator::Error::operator==(const Error& other) const {
  return message == other.message && type == other.type && severity == other.severity;
}

bool Validator::Error::operator!=(const Error& other) const { return !(*this == other); }

}  // namespace parser
}  // namespace maliput_sparse
